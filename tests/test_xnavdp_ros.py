"""Isolated native ROS protocol tests; opt-in only, never on the live domain."""

import copy
import json
import math
import os
import threading
import time

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get("DIMOS_XNAVDP_INTEGRATION_TEST") != "1",
    reason="set DIMOS_XNAVDP_INTEGRATION_TEST=1 in isolated ROS domain 173",
)


@pytest.fixture
def server():
    # Refuse synthetic sensor publication on the production domain.
    assert os.environ.get("ROS_DOMAIN_ID") == "173"
    import rclpy
    from dimos.plugins.xnavdp.client import EXPECTED_RELEASE, VERSION
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import CameraInfo, CompressedImage
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    context = Context()
    rclpy.init(
        context=context, domain_id=173, signal_handler_options=SignalHandlerOptions.NO
    )
    node = rclpy.create_node("xnavdp_protocol_test_server", context=context)
    state = {
        "initialized": True,
        "calibrated": True,
        "busy": False,
        "buffers": {"rgb": 20, "depth": 20, "odom": 20},
        "release": EXPECTED_RELEASE,
        "transport": VERSION,
        "resets": 0,
        "goals": 0,
    }
    rgb = node.create_publisher(
        CompressedImage,
        "/camera/camera/color/image_raw/compressed",
        qos_profile_sensor_data,
    )
    depth = node.create_publisher(
        CompressedImage,
        "/camera/camera/aligned_depth_to_color/image_raw/compressed",
        qos_profile_sensor_data,
    )
    odom = node.create_publisher(PoseStamped, "/odom", qos_profile_sensor_data)
    info = node.create_publisher(CameraInfo, "/realsense_camera_info", 10)
    path = node.create_publisher(Path, "/xnavdp/trajectory", 10)
    result_pub = node.create_publisher(String, "/xnavdp/result", 10)

    def sensors():
        image = CompressedImage()
        image.header.stamp = node.get_clock().now().to_msg()
        image.header.frame_id = "camera_color_optical_frame"
        image.data = b"fake-test-payload"
        rgb.publish(image)
        depth.publish(image)
        p = PoseStamped()
        p.header.stamp = image.header.stamp
        p.header.frame_id = "world"
        p.pose.position.x = 1.0
        p.pose.position.y = 2.0
        p.pose.orientation.z = math.sqrt(0.5)
        p.pose.orientation.w = math.sqrt(0.5)
        odom.publish(p)
        c = CameraInfo()
        c.header = image.header
        c.width = 848
        c.height = 480
        c.k = [430.0, 0.0, 417.0, 0.0, 429.0, 246.0, 0.0, 0.0, 1.0]
        info.publish(c)

    node.create_timer(0.05, sensors)

    def health(req, res):
        res.success = True
        res.message = json.dumps(state)
        return res

    def reset(req, res):
        state["resets"] += 1
        state["initialized"] = True
        res.success = True
        res.message = "reset"
        return res

    def goal(msg):
        from dimos.plugins.xnavdp.client import stamp_ns

        state["goals"] += 1
        coordinates = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [3.0, 0.0, 0.0]]
        result = Path()
        result.header = copy.deepcopy(msg.header)
        result.header.frame_id = "base_link"
        for x, y, z in coordinates:
            point = PoseStamped()
            point.header = copy.deepcopy(result.header)
            point.pose.position.x, point.pose.position.y, point.pose.position.z = (
                x,
                y,
                z,
            )
            point.pose.orientation.w = 1.0
            result.poses.append(point)
        payload = {
            "status": "success",
            "request_stamp_ns": stamp_ns(msg.header),
            "trajectory": [coordinates],
            "all_trajectory": [[coordinates]],
            "all_values": [[0.25]],
            "trajectory_frame": "base_link",
            "goal_frame": "base_link",
            "world_frame": "world",
            "goal": [3.0, 0.0],
            "robot_pos": [1.0, 2.0, 0.0],
            "robot_quat": [0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)],
            "odom_rgb_gap_ms": 1.0,
        }
        result_pub.publish(String(data=json.dumps(payload)))
        path.publish(result)

    node.create_service(Trigger, "/xnavdp/health", health)
    node.create_service(Trigger, "/xnavdp/reset", reset)
    node.create_subscription(PoseStamped, "/xnavdp/point_goal", goal, 5)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        yield node, state
    finally:
        executor.shutdown(timeout_sec=2)
        thread.join(timeout=2)
        node.destroy_node()
        context.try_shutdown()


def make_client():
    from dimos.plugins.xnavdp.client import XNavDPClient

    return XNavDPClient(
        {"sensor_timeout_s": 5.0, "service_timeout_s": 5.0, "response_timeout_s": 5.0}
    ).start()


def test_health_has_no_goal_or_duplicate_info_publisher(server):
    client = make_client()
    try:
        assert client.health()["initialized"]
        assert client.goal_pub is None
        assert client.node.count_publishers("/xnavdp/point_goal") == 0
        deadline = time.monotonic() + 3
        while (
            client.node.count_publishers("/realsense_camera_info") < 1
            and time.monotonic() < deadline
        ):
            time.sleep(0.05)
        assert client.node.count_publishers("/realsense_camera_info") == 1
        assert client.node.count_publishers("/cmd_vel") == 0
    finally:
        client.stop()
    assert server[1]["goals"] == 0 and server[1]["resets"] == 0


def test_native_request_and_result_roundtrip(server):
    from dimos.plugins.xnavdp.client import stamp_ns

    client = make_client()
    try:
        client.prepare(False)
        client.open_requests()
        goal = client.next_goal([3.0, 0.0])
        assert [goal.pose.position.x, goal.pose.position.y] == pytest.approx([1.0, 5.0])
        timing = {}
        parts, elapsed = client.infer(goal, timing_ms=timing)
        assert len(parts["path"].poses) == 3 and elapsed > 0
        assert stamp_ns(parts["path"].header) == stamp_ns(goal.header)
        assert timing["xnavdp_discovery"] >= 0
        assert timing["xnavdp_health"] > 0
        assert 0 < timing["xnavdp_response"] <= elapsed * 1000
        assert timing["xnavdp_validation"] > 0
        assert client.node.count_publishers("/cmd_vel") == 0
    finally:
        client.stop()
    assert server[1]["goals"] == 1 and server[1]["resets"] == 0


def test_preview_runner_discovers_planning_endpoints_before_any_request(server):
    from dimos.plugins.vln_task.assembly import create_runner
    from dimos.plugins.vln_task.profile import load_profile

    profile = load_profile("config/profiles/vln_preview.yaml")
    node, state = server
    with create_runner(profile):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            if (
                node.count_publishers("/xnavdp/point_goal") == 1
                and node.count_subscribers("/xnavdp/trajectory") == 1
                and node.count_subscribers("/xnavdp/result") == 1
            ):
                break
            time.sleep(0.05)
        else:
            pytest.fail("Planning endpoints were not discovered before the first task")
        assert node.count_publishers("/cmd_vel") == 0
        assert state["goals"] == 0 and state["resets"] == 0


def test_reset_requires_explicit_parameter(server):
    server[1]["initialized"] = False
    client = make_client()
    try:
        with pytest.raises(RuntimeError, match="reset_if_needed is false"):
            client.prepare(False)
        assert server[1]["resets"] == 0
        client.prepare(True)
        assert server[1]["resets"] == 1
        client.prepare(True)
        assert server[1]["resets"] == 1
    finally:
        client.stop()


def test_other_request_publisher_prevents_active_test(server):
    from geometry_msgs.msg import PoseStamped

    other = server[0].create_publisher(PoseStamped, "/xnavdp/point_goal", 5)
    client = make_client()
    try:
        client.health()
        deadline = time.monotonic() + 3
        while (
            client.node.count_publishers("/xnavdp/point_goal") == 0
            and time.monotonic() < deadline
        ):
            time.sleep(0.05)
        assert not client.warmup_requests()
        assert client.node.count_publishers("/xnavdp/point_goal") == 1
        with pytest.raises(RuntimeError, match="Another point-goal publisher"):
            client.open_requests()
        assert server[1]["goals"] == 0
    finally:
        client.stop()
        server[0].destroy_publisher(other)


def test_missing_inputs_prevent_goal_and_reset(server):
    server[1]["buffers"] = {"rgb": 0, "depth": 0, "odom": 0}
    client = make_client()
    try:
        with pytest.raises(RuntimeError, match="missing RGB/depth/odom"):
            client.prepare(True)
        assert client.goal_pub is None
        assert server[1]["goals"] == 0 and server[1]["resets"] == 0
    finally:
        client.stop()
