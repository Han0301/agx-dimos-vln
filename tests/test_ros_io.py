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
