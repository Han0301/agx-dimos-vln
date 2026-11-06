#!/usr/bin/env python3
"""Supply visualization-only camera TF when no robot TF is available."""
import math
import time
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster


def camera_mount():
    config = yaml.safe_load((Path(__file__).resolve().parents[3] / 'dimos/agents/skills/config/vln_config.yaml').read_text())
    navdp = config['navdp']
    name = navdp['trajectory_camera']
    mount = navdp['cameras'].get(name)
    if mount is None:
        raise ValueError(f'camera mount not found for {name}')
    return mount



def transform(mount, stamp):
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = 'base_link'
    t.child_frame_id = 'camera_color_optical_frame'
    t.transform.translation.x = float(mount['x'])
    t.transform.translation.y = float(mount['y'])
    t.transform.translation.z = float(mount['z'])
    # R_y(downward pitch) * optical-to-body rotation.
    # Optical (right, down, forward) -> body (forward, left, up).
    s, c = math.sin(float(mount['pitch']) / 2), math.cos(float(mount['pitch']) / 2)
    q = t.transform.rotation
    q.x, q.y, q.z, q.w = -(c+s)/2, (c+s)/2, (s-c)/2, (c-s)/2
    return t


def main():
    rclpy.init()
    node = Node('xnavdp_camera_visualization_tf')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    broadcaster = StaticTransformBroadcaster(node)
    deadline = time.monotonic() + 3
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if buffer.can_transform('base_link', 'camera_color_optical_frame', Time()):
        node.get_logger().info('Using existing camera -> base_link TF')
    else:
        mount = camera_mount()
        broadcaster.sendTransform(transform(mount, node.get_clock().now().to_msg()))
        node.get_logger().warning(
            f'Visualization TF from selected trajectory camera: {mount["x"]}, {mount["y"]}, {mount["z"]} m; '
            f'pitch={mount["pitch"]} rad. Alignment depends on this configured mount matching the real camera.')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
