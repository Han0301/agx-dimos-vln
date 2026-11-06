#!/usr/bin/env python3
"""ROS2 sensor subscribers and standard, inspectable navigation messages.

Binary framing is used ONLY for the on-host Python ABI bridge to the model.
It never crosses ROS or the network.
"""
import copy
import json
import math
import os
from pathlib import Path
import subprocess
import threading
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor

import cv2
import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path as ROSPath
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from local_model_protocol import pack, unpack, read_frame, write_frame

VERSION = 'xnavdp-ros2-topics-v2'

def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

class SensorCache:
    """Two seconds by local arrival time; odom-anchored nearest timestamp match."""
    def __init__(self):
        self.data = {key: deque() for key in ('rgb', 'depth', 'odom')}
        self.received = {key: 0 for key in self.data}
        self.last_arrival = {key: None for key in self.data}
        self.max_arrival_gap = {key: 0.0 for key in self.data}

    def trim(self):
        cutoff = time.monotonic() - 2.0
        for queue in self.data.values():
            while queue and queue[0][0] < cutoff:
                queue.popleft()

    def add(self, key, message):
        self.trim()
        now = time.monotonic()
        previous = self.last_arrival[key]
        if previous is not None:
            self.max_arrival_gap[key] = max(self.max_arrival_gap[key], now - previous)
        self.last_arrival[key] = now
        self.received[key] += 1
        self.data[key].append((now, stamp_ns(message), message))

    def nearest(self, key, ns):
        return self.nearest_with_gap(key, ns)[0]

    def nearest_with_gap(self, key, ns):
        self.trim()
        queue = self.data[key]
        if not queue:
            return None, None
        _, timestamp, message = min(queue, key=lambda item: abs(item[1] - ns))
        gap = abs(timestamp - ns)
        return message, gap

    def match_details(self, ns):
        odom, goal_odom_gap = self.nearest_with_gap('odom', ns)
        if odom is None:
            return None, {}
        odom_stamp = stamp_ns(odom)
        rgb, odom_rgb_gap = self.nearest_with_gap('rgb', odom_stamp)
        if rgb is None:
            return None, {}
        rgb_stamp = stamp_ns(rgb)
        depth, rgb_depth_gap = self.nearest_with_gap('depth', rgb_stamp)
        if depth is None:
            return None, {}
        details = {
            'goal_odom_gap_ns': goal_odom_gap,
            'odom_rgb_gap_ns': odom_rgb_gap,
            'rgb_depth_gap_ns': rgb_depth_gap,
        }
        return (rgb, depth, odom), details

    def match(self, ns):
        return self.match_details(ns)[0]

    def debug_state(self):
        self.trim()
        state = {
            'buffers': {key: len(queue) for key, queue in self.data.items()},
            'received': dict(self.received),
            'max_arrival_gap_ms': {
                key: round(value * 1000, 3) for key, value in self.max_arrival_gap.items()
            },
            'last_arrival_age_ms': {
                key: None if stamp is None else round((time.monotonic() - stamp) * 1000, 3)
                for key, stamp in self.last_arrival.items()
            },
            'cache_stamp_span_ms': {},
            'last_stamp_ns': {},
        }
        for key, queue in self.data.items():
            state['last_stamp_ns'][key] = queue[-1][1] if queue else None
            state['cache_stamp_span_ms'][key] = (
                round((queue[-1][1] - queue[0][1]) / 1_000_000, 3) if queue else None
            )
        return state

class Server(Node):
    def __init__(self):
        super().__init__('xnavdp_sensor_server')
        root = Path(__file__).resolve().parent
        env = dict(os.environ)
        env['PYTHONPATH'] = os.environ['MODEL_SOURCE'] + ':' + os.environ['XNAVDP_MODEL_ROOT']
        env.pop('PYTHONHOME', None)
        self.worker = subprocess.Popen([os.environ['XNAVDP_PYTHON'], '-u', str(root/'model_worker.py')], stdin=subprocess.PIPE, stdout=subprocess.PIPE, env=env)
        self.worker_lock = threading.Lock()
        self.owner = 'ros2_sensor_server'
        self.cache = SensorCache()
        self.intrinsic = None
        self.pending = None
        self.busy = False
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.health = self.rpc('health')[0]['health']
        self.health['transport'] = VERSION
        self.path_pub = self.create_publisher(ROSPath, '/xnavdp/model_path', 5)
        self.candidates_pub = self.create_publisher(MarkerArray, '/xnavdp/model_candidates', 5)
        self.pose_pub = self.create_publisher(PoseStamped, '/xnavdp/inference_pose', 5)
        self.status_pub = self.create_publisher(DiagnosticArray, '/xnavdp/status', 5)
        self.subs = []
        for key, topic, typ in [
            ('rgb', os.environ.get('XNAVDP_RGB_TOPIC', '/camera/camera/color/image_raw/compressed'), CompressedImage),
            ('depth', os.environ.get('XNAVDP_DEPTH_TOPIC', '/camera/camera/aligned_depth_to_color/image_raw/compressed'), CompressedImage),
            ('odom', '/odom', PoseStamped),
        ]:
            self.subs.append(self.create_subscription(typ, topic, lambda msg, k=key: self.cache.add(k, msg), qos_profile_sensor_data))
            self.get_logger().info(f'Subscribing {topic} ({typ.__name__})')
        self.subs.append(self.create_subscription(CameraInfo, '/realsense_camera_info', self.calibration, 10))
        self.subs.append(self.create_subscription(PoseStamped, '/xnavdp/point_goal', self.goal, 5))
        self.health_srv = self.create_service(Trigger, '/xnavdp/health', self.on_health)
        self.reset_srv = self.create_service(Trigger, '/xnavdp/reset', self.on_reset)
        self.timer = self.create_timer(0.02, self.dispatch)
        self.get_logger().info('ROS2 XNavDP: independent sensor topics, world point goals, world model paths')

    def rpc(self, operation, **arrays):
        with self.worker_lock:
            if self.worker.poll() is not None:
                raise RuntimeError('Model worker exited; restart server')
            write_frame(self.worker.stdin, pack({'id': str(time.monotonic_ns()), 'owner': self.owner, 'op': operation}, **arrays))
            metadata, result = unpack(read_frame(self.worker.stdout))
            if not metadata.get('ok'):
                raise RuntimeError(metadata.get('error', 'Model operation failed'))
            return metadata, result

    def calibration(self, msg):
        self.intrinsic = np.asarray(msg.k).reshape(3, 3)

    def on_health(self, request, response):
        state = dict(self.health, busy=self.busy, calibrated=self.intrinsic is not None,
                     **self.cache.debug_state())
        response.success = self.worker.poll() is None
        response.message = json.dumps(state)
        return response

    def on_reset(self, request, response):
        if self.busy or self.pending:
            response.success, response.message = False, 'Inference is active; stop navigation before resetting'
        elif self.intrinsic is None:
            response.success, response.message = False, 'Waiting for /realsense_camera_info'
        else:
            try:
                self.rpc('reset', intrinsic=self.intrinsic)
                self.health['initialized'] = True
                response.success, response.message = True, 'Model history reset'
            except Exception as exc:
                response.success, response.message = False, str(exc)
        return response

    def report(self, header, error, **values):
        status = DiagnosticStatus(name='xnavdp/inference', level=DiagnosticStatus.ERROR if error else DiagnosticStatus.OK,
                                  message=str(error) if error else 'OK', values=[KeyValue(key=k, value=str(v)) for k, v in values.items()])
        self.status_pub.publish(DiagnosticArray(header=header, status=[status]))

    def goal(self, msg):
        if self.busy or self.pending:
            self.report(msg.header, 'Previous inference still active')
            return
        if not self.health.get('initialized'):
            self.report(msg.header, 'Call /xnavdp/reset before inference')
            return
        self.pending = (msg, time.monotonic())

    def dispatch(self):
        if self.pending is None or self.busy:
            return
        goal, received = self.pending
        goal_stamp = stamp_ns(goal)
        matched, details = self.cache.match_details(goal_stamp)
        if matched is None:
            self.pending = None
            return
        slow_gaps = {
            key: round(value / 1e6, 3)
            for key, value in details.items()
            if value is not None and value > 150_000_000
        }
        if slow_gaps:
            self.get_logger().warning(
                f'Sensor match exceeded 150 ms but will still run: {slow_gaps}')
        self.pending, self.busy = None, True
        self.pool.submit(self.infer, goal, matched, details)

    def infer(self, goal, matched, match_details):
        started = time.monotonic()
        try:
            rgb, depth_msg, odom = matched
            if goal.header.frame_id != odom.header.frame_id:
                raise ValueError(f'Goal frame {goal.header.frame_id!r} differs from odom frame {odom.header.frame_id!r}')
            q = odom.pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            c, s = math.cos(yaw), math.sin(yaw)
            rotation = np.array([[c,-s,0],[s,c,0],[0,0,1]])
            position = np.array([odom.pose.position.x, odom.pose.position.y, 0.0])
            target = np.array([goal.pose.position.x, goal.pose.position.y, 0.0])
            local = ((target-position) @ rotation)[:2]
            raw_depth = np.frombuffer(bytes(depth_msg.data), np.uint8)
            # Support both plain PNG and ROS compressedDepth's 12-byte prefix.
            offset = bytes(raw_depth[:20]).find(b'\x89PNG\r\n\x1a\n')
            decoded = cv2.imdecode(raw_depth[max(offset,0):], cv2.IMREAD_UNCHANGED)
            if decoded is None:
                raise ValueError('Cannot decode compressed depth PNG')
            scaled = np.clip(decoded.astype(np.float32) * (10000.0 / float(os.environ.get('XNAVDP_DEPTH_SCALE', '1000.0'))), 0, 65535).astype(np.uint16)
            ok, png = cv2.imencode('.png', scaled)
            if not ok:
                raise ValueError('Cannot encode model depth')
            _, arrays = self.rpc('step', image=np.frombuffer(bytes(rgb.data), np.uint8), depth=png,
                                  goal=local, position=position, quaternion=np.array([0,0,math.sin(yaw/2),math.cos(yaw/2)]))
            # All ROS outputs use the goal's requested RGB timestamp as correlation key.
            header = copy.deepcopy(goal.header)
            reference = copy.deepcopy(odom)
            reference.header = header
            reference.pose.position.z = 0.0
            reference.pose.orientation.x = reference.pose.orientation.y = 0.0
            reference.pose.orientation.z, reference.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            self.pose_pub.publish(reference)
            trajectory = np.asarray(arrays['trajectory'])[0] @ rotation.T + position
            path = ROSPath(header=header)
            for xyz in trajectory:
                pose = PoseStamped(header=header)
                pose.pose.position = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
            markers = MarkerArray()
            for index, candidate in enumerate(arrays['candidates'][0]):
                marker = Marker(header=header, ns='xnavdp_model', id=index, type=Marker.LINE_STRIP, action=Marker.ADD)
                marker.pose.orientation.w = 1.0
                marker.scale.x = .025
                marker.color.r, marker.color.g, marker.color.a = .2, .8, .7
                marker.text = str(float(np.asarray(arrays['values']).reshape(-1)[index]))
                marker.lifetime.sec = 2
                marker.points = [Point(x=float(p[0]),y=float(p[1]),z=float(p[2])) for p in candidate @ rotation.T + position]
                markers.markers.append(marker)
            self.candidates_pub.publish(markers)
            self.path_pub.publish(path)
            self.report(header, None, rgb_stamp_ns=stamp_ns(rgb), depth_stamp_ns=stamp_ns(depth_msg), odom_stamp_ns=stamp_ns(odom),
                        local_goal=local.tolist(), roundtrip_seconds=time.monotonic()-started,
                        goal_odom_gap_ms=round(match_details['goal_odom_gap_ns'] / 1e6, 3),
                        odom_rgb_gap_ms=round(match_details['odom_rgb_gap_ns'] / 1e6, 3),
                        rgb_depth_gap_ms=round(match_details['rgb_depth_gap_ns'] / 1e6, 3))
        except Exception as exc:
            self.get_logger().error(str(exc))
            self.report(goal.header, exc)
        finally:
            self.busy = False

    def destroy_node(self):
        self.pool.shutdown(wait=True, cancel_futures=True)
        self.worker.stdin.close()
        try:
            self.worker.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.worker.terminate()
            self.worker.wait(timeout=5)
        return super().destroy_node()

def main():
    rclpy.init()
    node = Server()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
if __name__ == '__main__': main()
