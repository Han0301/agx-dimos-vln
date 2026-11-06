"""One-module ROS body bridge, with the existing camera calibration only."""

import os

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from dimos.agents.skills.config import load_vln_config
from dimos.core.blueprints import autoconnect
from dimos.core.transport import ROSTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.go2.body_bridge import Go2BodyBridge

_camera = load_vln_config(os.environ.get("VLN_CONFIG")).navdp.get_trajectory_camera()

unitree_go2_body_ros = (
    autoconnect(Go2BodyBridge.blueprint())
    .transports({
        ("odom", PoseStamped): ROSTransport(
            "/odom", PoseStamped,
            qos=QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=5,
            ),
        ),
        ("lidar", PointCloud2): ROSTransport("/lidar", PointCloud2),
        ("cmd_vel", Twist): ROSTransport("/cmd_vel", Twist),
        ("realsense_camera_info", CameraInfo): ROSTransport("/realsense_camera_info", CameraInfo),
    })
    .global_config(
        n_workers=1,
        viewer="none",
        robot_model="unitree_go2",
        realsense_x=_camera.x,
        realsense_y=_camera.y,
        realsense_z=_camera.z,
        realsense_pitch=_camera.pitch,
        realsense_intrinsic=[v for row in _camera.intrinsic for v in row] if _camera.intrinsic else None,
        realsense_width=_camera.width,
        realsense_height=_camera.height,
    )
)

__all__ = ["unitree_go2_body_ros"]
