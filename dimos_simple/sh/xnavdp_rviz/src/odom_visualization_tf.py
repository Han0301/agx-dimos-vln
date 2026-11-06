#!/usr/bin/env python3
"""Visualization-only world/base TF from the native DimOS ROS /odom topic."""
import copy
import math
import time
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

class OdomVisualizationTF(Node):
    def __init__(self):
        super().__init__('xnavdp_odom_visualization_tf')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.deadline = time.monotonic() + 3.0
        self.mode = None
        self.sub = self.create_subscription(PoseStamped, '/odom', self.odom, 10)

    def odom(self, odom):
        if self.mode is None:
            if self.buffer.can_transform('world', 'base_link', Time()):
                self.mode = 'existing'
                self.get_logger().info('Using existing world -> base_link TF')
            elif time.monotonic() >= self.deadline:
                self.mode = 'ros'
                self.get_logger().info('Using ROS /odom for world -> base_link TF')
            else:
                return
        if self.mode != 'ros':
            return
        q = odom.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        msg = TransformStamped()
        msg.header = copy.deepcopy(odom.header)
        msg.child_frame_id = 'base_link'
        msg.transform.translation.x = odom.pose.position.x
        msg.transform.translation.y = odom.pose.position.y
        msg.transform.rotation.z = math.sin(yaw/2)
        msg.transform.rotation.w = math.cos(yaw/2)
        self.broadcaster.sendTransform(msg)

def main():
    rclpy.init()
    node = OdomVisualizationTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
if __name__ == '__main__': main()
