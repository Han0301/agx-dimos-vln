"""Fill missing visualization transforms from native odom and the project's mount."""

import math
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import (
    Buffer,
    StaticTransformBroadcaster,
    TransformBroadcaster,
    TransformListener,
)


def main():
    rclpy.init()
    node = Node("xnavdp_camera_visualization_tf")
    root = Path(__file__).resolve().parents[4]
    config = yaml.safe_load((root / "config/base.yaml").read_text())
    base, camera = config["body"]["base_frame"], config["camera"]["mount_frame"]
    optical = "camera_color_optical_frame"
    buffer = Buffer()
    _listener = TransformListener(buffer, node)
    static = StaticTransformBroadcaster(node)
    dynamic = TransformBroadcaster(node)
    latest = None
    supply_odom = False

    def transform(parent, child, stamp):
        result = TransformStamped()
        result.header.frame_id, result.child_frame_id = parent, child
        result.header.stamp = stamp
        result.transform.rotation.w = 1.0
        return result

    def receive(msg):
        nonlocal latest
        latest = msg
        if supply_odom:
            result = transform(msg.header.frame_id, base, msg.header.stamp)
            result.transform.translation.x = msg.pose.position.x
            result.transform.translation.y = msg.pose.position.y
            result.transform.translation.z = msg.pose.position.z
            result.transform.rotation = msg.pose.orientation
            dynamic.sendTransform(result)

    _subscription = node.create_subscription(
        PoseStamped, "/odom", receive, qos_profile_sensor_data
    )
    try:
        deadline = time.monotonic() + 3
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        stamp = node.get_clock().now().to_msg()
        transforms = []
        if not buffer.can_transform(base, optical, Time()):
            if not buffer.can_transform(base, camera, Time()):
                mount = transform(base, camera, stamp)
                (
                    mount.transform.translation.x,
                    mount.transform.translation.y,
                    mount.transform.translation.z,
                ) = config["camera"]["translation"]
                mount.transform.rotation.y = math.sin(config["camera"]["pitch"] / 2)
                mount.transform.rotation.w = math.cos(config["camera"]["pitch"] / 2)
                transforms.append(mount)
            if not buffer.can_transform(camera, optical, Time()):
                optical_tf = transform(camera, optical, stamp)
                q = optical_tf.transform.rotation
                q.x, q.y, q.z, q.w = -0.5, 0.5, -0.5, 0.5
                transforms.append(optical_tf)
        if transforms:
            static.sendTransform(transforms)
            node.get_logger().info(
                "Visualization-only missing camera TF uses config/base.yaml mount"
            )
        world = (
            latest.header.frame_id
            if latest is not None
            else config["body"]["world_frame"]
        )
        supply_odom = not buffer.can_transform(world, base, Time())
        if latest is not None and supply_odom:
            receive(latest)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
