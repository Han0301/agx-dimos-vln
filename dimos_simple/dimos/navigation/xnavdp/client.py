"""ROS2 point-goal client. Sensors travel on their own standard ROS topics."""
from __future__ import annotations
import json
import math
import os
import threading
import time
import uuid
from pathlib import Path
import numpy as np

VERSION = 'xnavdp-ros2-topics-v2'
EXPECTED_RELEASE = 'unified-20261104'

class XNavDPClient:
    def __init__(self, url: str = '', *, debug_mode: bool = True):
        if url and not url.startswith('ros2://'):
            raise ValueError('XNavDP requires ros2:///xnavdp')
        os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
        profile = os.environ.get('XNAVDP_DDS_PROFILE', str(Path(__file__).resolve().parents[3] / 'sh/xnavdp_ros2/client_dds.xml'))
        if Path(profile).is_file():
            os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = profile
        os.environ.pop('ROS_LOCALHOST_ONLY', None)
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from diagnostic_msgs.msg import DiagnosticArray
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Path as ROSPath
        from sensor_msgs.msg import CameraInfo, CompressedImage
        from std_srvs.srv import Trigger
        from visualization_msgs.msg import MarkerArray
        from rclpy.qos import qos_profile_sensor_data
        self.rclpy = rclpy
        self.context = Context()
        rclpy.init(context=self.context, domain_id=int(os.environ.get('XNAVDP_ROS_DOMAIN_ID', '0')))
        self.node = Node('xnavdp_client_' + uuid.uuid4().hex[:8], context=self.context)
        self.url = 'ros2:///xnavdp'
        self.pose_type, self.info_type, self.trigger_type = PoseStamped, CameraInfo, Trigger
        self.goal_pub = self.node.create_publisher(PoseStamped, '/xnavdp/point_goal', 5)
        self.info_pub = self.node.create_publisher(CameraInfo, '/realsense_camera_info', 10)
        self.health_client = self.node.create_client(Trigger, '/xnavdp/health')
        self.reset_client = self.node.create_client(Trigger, '/xnavdp/reset')
        self.lock = threading.Lock()
        self.event = threading.Event()
        self.pending = None
        self.parts = {}
        self.error = None
        self.latest_rgb = None
        self.latest_odom = None
        self.subscriptions = [
            self.node.create_subscription(ROSPath, '/xnavdp/model_path', lambda msg: self._receive('path', msg, msg.header), 5),
            self.node.create_subscription(PoseStamped, '/xnavdp/inference_pose', lambda msg: self._receive('pose', msg, msg.header), 5),
            self.node.create_subscription(MarkerArray, '/xnavdp/model_candidates', self._candidates, 5),
            self.node.create_subscription(DiagnosticArray, '/xnavdp/status', self._status, 5),
            self.node.create_subscription(CompressedImage, os.environ.get('XNAVDP_RGB_TOPIC', '/camera/camera/color/image_raw/compressed'), self._rgb, qos_profile_sensor_data),
            self.node.create_subscription(PoseStamped, '/odom', self._odom, qos_profile_sensor_data),
        ]
        self.initialized = False
        self.debug_mode = debug_mode
        self.last_trace_id = ''
        self.last_result_at_ns = 0
        self.last_http_seconds = 0.0  # compatibility name: ROS2 round-trip
        self.last_pose = None
        self.last_local_goal = None
        self.health = {}
        self.closed = False
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    @staticmethod
    def _stamp(header):
        return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec

    def _rgb(self, msg):
        self.latest_rgb = msg

    def _odom(self, msg):
        self.latest_odom = msg

    def _receive(self, key, msg, header):
        if self._stamp(header) == self.pending:
            self.parts[key] = msg
            if all(k in self.parts for k in ('path', 'pose', 'candidates')):
                self.event.set()

    def _candidates(self, msg):
        if msg.markers:
            self._receive('candidates', msg, msg.markers[0].header)

    def _status(self, msg):
        if self._stamp(msg.header) == self.pending:
            errors = [s.message for s in msg.status if (int.from_bytes(s.level, 'little') if isinstance(s.level, bytes) else int(s.level)) >= 2]
            if errors:
                self.error = '; '.join(errors)
                self.event.set()

    def _service(self, client, timeout):
        if not client.wait_for_service(timeout_sec=min(timeout, 10)):
            raise TimeoutError(f'ROS2 service {client.srv_name} unavailable; check DDS/network')
        done = threading.Event()
        future = client.call_async(self.trigger_type.Request())
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            client.remove_pending_request(future)
            raise TimeoutError(f'ROS2 service {client.srv_name} timed out')
        result = future.result()
        if result is None or not result.success:
            raise RuntimeError(result.message if result else str(future.exception()))
        return result.message

    def verify_backend(self):
        self.health = json.loads(self._service(self.health_client, 10))
        if self.health.get('release') != EXPECTED_RELEASE or self.health.get('transport') != VERSION:
            raise ValueError(f'Unexpected XNavDP ROS2 backend: {self.health}')

    def attach_initialized_backend(self):
        """Use an existing model without changing calibration or model history."""
        self.initialized = False
        self.verify_backend()
        if not self.health.get('initialized'):
            raise RuntimeError('XNavDP model is not initialized; initialize it with camera calibration first')
        self.initialized = True
        return True

    def initialize(self, intrinsic):
        info = self.info_type()
        info.header.frame_id = 'camera_color_optical_frame'
        info.k = np.asarray(intrinsic, dtype=float).reshape(9).tolist()
        end = time.monotonic()+10
        while self.info_pub.get_subscription_count() == 0:
            if time.monotonic() > end:
                raise TimeoutError('Server not subscribed to /realsense_camera_info')
            time.sleep(.05)
        self.info_pub.publish(info)
        # Wait for the calibration callback before calling the reset service.
        while True:
            self.verify_backend()
            if self.health.get('calibrated'):
                break
            if time.monotonic() > end:
                raise TimeoutError('Server has no camera calibration')
            self.info_pub.publish(info)
            time.sleep(.05)
        self._service(self.reset_client, 180)
        self.initialized = True
        return True

    def reset(self, intrinsic):
        self.initialized = False
        return self.initialize(intrinsic)

    def pointgoal_step(self, rgb, depth, goal, position, quaternion, *, stamp=None, frame_id='world'):
        """Publish world goal only; RGB/depth/odom are never bundled or sent here.

        rgb/depth are retained in the facade for existing navigator callers.
        stamp identifies the RGB observation to match in the server's 2 s cache.
        """
        if not self.initialized:
            raise RuntimeError('Call /xnavdp/reset before inference')
        if stamp is None:
            raise ValueError('The RGB ROS timestamp is required for sensor matching')
        with self.lock:
            started = time.monotonic()
            ns = int(round(stamp * 1e9))
            q = np.asarray(quaternion)
            yaw = math.atan2(2*(q[3]*q[2]+q[0]*q[1]), 1-2*(q[1]**2+q[2]**2))
            c, s = math.cos(yaw), math.sin(yaw)
            rotation = np.array([[c,-s,0],[s,c,0],[0,0,1]])
            local = np.array([goal[0],goal[1],0.0])
            target = local @ rotation.T + np.asarray(position)
            msg = self.pose_type()
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 1_000_000_000)
            msg.header.frame_id = frame_id
            msg.pose.position.x, msg.pose.position.y = float(target[0]), float(target[1])
            msg.pose.orientation.w = 1.0
            self.pending, self.parts, self.error = ns, {}, None
            self.event.clear()
            self.last_trace_id = str(ns)
            self.goal_pub.publish(msg)
            if not self.event.wait(60):
                self.pending = None
                raise TimeoutError('No matching ROS2 model path/pose/candidates after 60 s')
            self.pending = None
            if self.error:
                raise RuntimeError(self.error)
            reference = self.parts['pose'].pose
            q = reference.orientation
            yaw = math.atan2(2*q.w*q.z, 1-2*q.z*q.z)
            c, s = math.cos(yaw), math.sin(yaw)
            rotation = np.array([[c,-s,0],[s,c,0],[0,0,1]])
            position = np.array([reference.position.x,reference.position.y,reference.position.z])
            trajectory = np.array([[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in self.parts['path'].poses])
            markers = sorted(self.parts['candidates'].markers, key=lambda m:m.id)
            candidates = np.array([[[p.x,p.y,p.z] for p in m.points] for m in markers])
            values = np.array([[float(m.text) for m in markers]])
            arrays = ((trajectory-position) @ rotation, (candidates-position) @ rotation, values)
            traj, candidates, values = arrays[0][None], arrays[1][None], arrays[2]
            if (traj.ndim != 3 or traj.shape[0] != 1 or traj.shape[2] != 3 or not traj.shape[1]
                    or candidates.ndim != 4 or candidates.shape[0] != 1 or candidates.shape[2:] != traj.shape[1:]
                    or values.size != candidates.shape[1] or not all(np.isfinite(a).all() for a in (traj,candidates,values))):
                raise ValueError('Invalid ROS2 model paths/candidates/scores')
            self.last_pose = (position, rotation)
            self.last_local_goal = ((target-position) @ rotation)[:2]
            self.last_http_seconds = time.monotonic() - started
            self.last_result_at_ns = time.monotonic_ns()
            return traj,candidates,values

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.executor.shutdown(timeout_sec=2)
        self.thread.join(timeout=2)
        self.node.destroy_node()
        self.context.try_shutdown()
