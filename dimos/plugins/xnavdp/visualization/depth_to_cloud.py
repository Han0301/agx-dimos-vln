#!/usr/bin/env python3
"""Project rectified/aligned ROS depth images into XYZ optical-frame clouds."""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


def decode(msg):
    enc = msg.encoding.upper()
    dtype = {"16UC1": "u2", "MONO16": "u2", "32FC1": "f4"}.get(enc)
    if dtype is None:
        raise ValueError(f"Unsupported depth encoding: {msg.encoding}")
    dt = np.dtype((">" if msg.is_bigendian else "<") + dtype)
    if msg.step < msg.width * dt.itemsize or len(msg.data) < msg.step * msg.height:
        raise ValueError("Truncated depth image")
    raw = np.ndarray(
        (msg.height, msg.width),
        dtype=dt,
        buffer=bytes(msg.data),
        strides=(msg.step, dt.itemsize),
    )
    return raw.astype(np.float32) * (0.001 if dtype == "u2" else 1.0)


def project(depth, info, stride, max_depth):
    if (info.height, info.width) != depth.shape:
        raise ValueError("Depth and CameraInfo sizes differ")
    # P is the rectified projection matrix; aligned RealSense depth is rectified.
    fx, fy, cx, cy = info.p[0], info.p[5], info.p[2], info.p[6]
    if not fx or not fy:
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    if not np.isfinite([fx, fy, cx, cy]).all() or fx <= 0 or fy <= 0:
        raise ValueError("Invalid CameraInfo intrinsics")
    v, u = np.mgrid[0 : depth.shape[0] : stride, 0 : depth.shape[1] : stride]
    z = depth[::stride, ::stride]
    good = np.isfinite(z) & (z > 0) & (z <= max_depth)
    return np.column_stack(
        ((u[good] - cx) * z[good] / fx, (v[good] - cy) * z[good] / fy, z[good])
    ).astype("<f4")


class DepthToCloud(Node):
    def __init__(self):
        super().__init__("xnavdp_depth_to_cloud")
        defaults = {
            "depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
            "camera_info_topic": "/realsense_camera_info",
            "output_topic": "/xnavdp/depth/points",
            "pixel_stride": 2,
            "max_depth": 8.0,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)

        def val(key):
            return self.get_parameter(key).value

        self.stride, self.max_depth = int(val("pixel_stride")), float(val("max_depth"))
        if self.stride < 1 or not np.isfinite(self.max_depth) or self.max_depth <= 0:
            raise ValueError("pixel_stride and max_depth must be positive")
        self.info = None
        self.last_warning = None
        self.count = 0
        self.pub = self.create_publisher(
            PointCloud2, val("output_topic"), qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo, val("camera_info_topic"), self.on_info, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, val("depth_topic"), self.on_depth, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"Depth -> {val('output_topic')}; waiting for depth and CameraInfo"
        )

    def on_info(self, msg):
        self.info = msg

    def warn(self, text):
        if text != self.last_warning:
            self.get_logger().warning(text)
            self.last_warning = text

    def on_depth(self, msg):
        if self.info is None:
            self.warn("Waiting for CameraInfo; no guessed intrinsics are used")
            return
        if (
            msg.header.frame_id
            and self.info.header.frame_id
            and msg.header.frame_id != self.info.header.frame_id
        ):
            self.warn("Depth and CameraInfo frame_id differ; check topic pairing")
            return
        try:
            xyz = project(decode(msg), self.info, self.stride, self.max_depth)
        except (ValueError, TypeError) as exc:
            self.warn(str(exc))
            return
        cloud = PointCloud2()
        cloud.header = msg.header
        cloud.height, cloud.width = 1, len(xyz)
        cloud.fields = [
            PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
            for i, name in enumerate(("x", "y", "z"))
        ]
        cloud.is_bigendian = False
        cloud.point_step, cloud.row_step = 12, len(xyz) * 12
        cloud.is_dense = True
        cloud.data = xyz.tobytes()
        self.pub.publish(cloud)
        self.count += 1
        if self.count == 1:
            self.get_logger().info(
                f"Publishing {len(xyz)} points, frame={cloud.header.frame_id}, encoding={msg.encoding}"
            )


def main():
    rclpy.init()
    node = DepthToCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
