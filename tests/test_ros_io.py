"""Native ROS message compatibility, without starting a robot or a ROS node."""

import json
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip("geometry_msgs.msg")
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped

from dimos.core.config import ROOT, load_config
from dimos.robot.unitree.go2.body_bridge import Go2BodyBridge


class Sink:
    def __init__(self):
        self.items = []

    def publish(self, msg):
        self.items.append(msg)


def test_native_body_messages_and_single_tf_ownership():
    config = load_config(ROOT / "config/base.yaml")
    node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=123, nanosec=456))
        )
    )
    bridge = Go2BodyBridge(node, config)
    bridge.pose_type = PoseStamped
    bridge.odom = Sink()
    bridge.tf = Sink()
    bridge.lidar = Sink()
    bridge.lowstate = Sink()
    bridge._on_odom(
        {
            "data": {
                "pose": {
                    "position": {"x": 1.0, "y": 2.0, "z": 0.3},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                }
            }
        }
    )
    pose = bridge.odom.items[0]
    assert pose.header.frame_id == "world" and pose.header.stamp.sec == 123
    assert pose.pose.position.x == 1.0
    frames = [
        (t.header.frame_id, t.child_frame_id) for t in bridge.tf.items[0].transforms
    ]
    assert frames == [("world", "base_link"), ("base_link", "camera_link")]
    points = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)
    bridge._on_lidar({"data": {"data": {"points": points}}})
    cloud = bridge.lidar.items[0]
    assert (
        cloud.header.frame_id == "world"
        and cloud.point_step == 12
        and cloud.row_step == 24
    )
    assert [field.name for field in cloud.fields] == ["x", "y", "z"]
    np.testing.assert_array_equal(
        np.frombuffer(cloud.data, dtype="<f4").reshape(-1, 3), points
    )
    bridge._on_lowstate({"data": {"battery": 50}})
    assert json.loads(bridge.lowstate.items[0].data) == {"data": {"battery": 50}}



def test_lowstate_forwards_rpy_as_imu():
    from sensor_msgs.msg import Imu

    stamp = Time(sec=123, nanosec=456)
    node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: stamp)
        )
    )
    bridge = Go2BodyBridge(node, load_config(ROOT / "config/base.yaml"))
    bridge.lowstate = Sink()
    bridge.imu = Sink()

    payload = {"data": {"imu_state": {"rpy": [0.0, 0.0, np.pi / 2]}}}
    bridge._on_lowstate(payload)
    bridge._on_lowstate({"data": {"imu_state": {"rpy": [0.0, 1.0]}}})
    bridge._on_lowstate({"data": {"battery": 50}})

    assert len(bridge.lowstate.items) == 3
    assert json.loads(bridge.lowstate.items[0].data) == payload
    assert len(bridge.imu.items) == 1
    imu = bridge.imu.items[0]
    assert isinstance(imu, Imu)
    assert imu.header.frame_id == "base_link"
    assert imu.header.stamp == stamp
    orientation = imu.orientation
    assert [orientation.x, orientation.y, orientation.z, orientation.w] == pytest.approx(
        [0.0, 0.0, 0.70710678, 0.70710678]
    )
    assert imu.angular_velocity_covariance[0] == -1.0
    assert imu.linear_acceleration_covariance[0] == -1.0
