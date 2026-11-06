#!/usr/bin/env python3
"""Convert XNavDP JSON results to RViz markers, without changing inference."""
import json
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class ResultVisualizer(Node):
    def __init__(self):
        super().__init__('xnavdp_result_visualizer')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, '/xnavdp/result_markers', qos)
        self.sub = self.create_subscription(String, '/xnavdp/result', self.receive, 10)
        self.get_logger().info('Visualizing /xnavdp/result on /xnavdp/result_markers')

    def convert(self, data):
        if data.get('trajectory_frame') != 'base_link' or data.get('goal_frame') != 'base_link':
            raise ValueError('Expected base_link result')
        position = [float(v) for v in data['robot_pos']]
        q = [float(v) for v in data['robot_quat']]
        if len(position) != 3 or len(q) != 4 or not all(math.isfinite(v) for v in position + q):
            raise ValueError('Invalid result pose')
        qnorm = math.sqrt(sum(v*v for v in q))
        if qnorm < 1e-9:
            raise ValueError('Invalid result quaternion')
        x, y, z, w = (v/qnorm for v in q)
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        c, sn = math.cos(yaw), math.sin(yaw)
        def world_point(xyz):
            a, b, h = (float(v) for v in xyz[:3])
            if not all(math.isfinite(v) for v in (a, b, h)):
                raise ValueError('Invalid point')
            return Point(x=position[0]+c*a-sn*b, y=position[1]+sn*a+c*b, z=position[2]+h)
        frame = 'world'
        stamp = self.get_clock().now().to_msg()
        markers = []

        def marker(namespace, index, kind):
            m = Marker()
            m.header.frame_id = frame
            m.header.stamp = stamp
            m.ns, m.id, m.type = namespace, index, kind
            m.action = Marker.ADD
            m.lifetime = Duration(sec=3)
            m.pose.orientation.w = 1.0
            m.color.a = 1.0
            markers.append(m)
            return m

        def line(namespace, index, points, width, color):
            m = marker(namespace, index, Marker.LINE_STRIP)
            m.scale.x = width
            m.color.r, m.color.g, m.color.b = color
            for xyz in points:
                vals = [float(v) for v in xyz[:3]]
                if len(vals) != 3 or not all(math.isfinite(v) for v in vals):
                    raise ValueError('Invalid trajectory point')
                m.points.append(world_point(vals))
            return m

        candidates = data.get('all_trajectory', [[]])[0]
        values = data.get('all_values', [[]])[0]
        for i, points in enumerate(candidates):
            line('candidates', i, points, 0.018, (0.25, 0.6, 1.0))
            if points and i < len(values):
                label = marker('scores', i, Marker.TEXT_VIEW_FACING)
                label.pose.position = world_point([float(points[-1][0]), float(points[-1][1]), float(points[-1][2]) + 0.12])
                label.scale.z = 0.09
                label.color.r = label.color.g = label.color.b = 1.0
                label.text = f'{i}: {float(values[i]):.3f}'
        trajectory = data.get('trajectory', [[]])[0]
        line('selected', 0, trajectory, 0.045, (0.15, 1.0, 0.3))
        goal = data.get('goal', [])
        if len(goal) >= 2:
            m = marker('goal', 0, Marker.SPHERE)
            m.pose.position = world_point([goal[0], goal[1], 0.05])
            m.scale.x = m.scale.y = m.scale.z = 0.16
            m.color.r, m.color.g, m.color.b = 1.0, 0.3, 0.15
        m = marker('info', 0, Marker.TEXT_VIEW_FACING)
        m.pose.position = world_point([0.0, 0.0, 0.65])
        m.scale.z = 0.12
        m.color.r = m.color.g = m.color.b = 1.0
        m.text = f"XNavDP | {len(candidates)} candidates | {data.get('transport', 'inference')} {float(data.get('roundtrip_seconds', data.get('http_seconds', 0))):.3f}s\n{data.get('pose_mode', '')} | green: selected, blue: candidates"
        clear = Marker()
        clear.action = Marker.DELETEALL
        return MarkerArray(markers=[clear] + markers)

    def receive(self, msg):
        try:
            self.pub.publish(self.convert(json.loads(msg.data)))
        except (ValueError, TypeError, KeyError, IndexError) as exc:
            self.get_logger().warning(f'Invalid result: {exc}')


def main():
    rclpy.init()
    node = ResultVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
