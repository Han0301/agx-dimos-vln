import json
import subprocess
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest
from dimos.config import ROOT
from dimos.plugins.path_tracker.config import load_config
from dimos.plugins.path_tracker.messages import decode_depth, decode_result
from dimos.plugins.path_tracker.mpc import solve_mpc
from dimos.plugins.path_tracker.output import VelocityOutput
from dimos.plugins.path_tracker.tracker import Tracker


def result_payload():
    trajectory = [[[0.2, 0.0, 0.0], [0.5, 0.0, 0.0], [1.0, 0.0, 0.0]]]
    return dict(
        status="success",
        request_stamp_ns=100000000001,
        trajectory=trajectory,
        all_trajectory=[trajectory],
        all_values=[[1.0]],
        robot_pos=[10.0, 2.0, 0.0],
        robot_quat=[0.0, 0.0, 0.0, 1.0],
        goal=[3.0, 0.0],
        trajectory_frame="base_link",
        goal_frame="base_link",
        world_frame="world",
    )


def test_config_check_has_no_ros_or_scipy():
    code = 'from dimos.plugins.path_tracker.__main__ import main; import sys; assert main(["--check"]) == 0; assert "rclpy" not in sys.modules; assert "scipy" not in sys.modules'
    subprocess.run(
        [sys.executable, "-c", code], cwd=ROOT, check=True, capture_output=True
    )


def test_output_only_never_creates_publisher(capsys):
    class ForbiddenNode:
        def create_publisher(self, *_):
            raise AssertionError("preview must not create a publisher")

    sink = VelocityOutput(ForbiddenNode(), "/cmd_vel", True)
    assert sink(0.4, 0.1, "tracking") is False
    assert json.loads(capsys.readouterr().out) == dict(
        linear_x=0.4, angular_z=0.1, reason="tracking", published=False
    )


def test_depth_padding_endianness_and_units():
    msg = NS(
        encoding="16UC1",
        is_bigendian=True,
        height=2,
        width=2,
        step=6,
        data=bytes.fromhex("03e807d000000bb80fa00000"),
    )
    np.testing.assert_allclose(decode_depth(msg, 1000), [[1, 2], [3, 4]])
    msg.data = msg.data[:3]
    with pytest.raises(ValueError, match="truncated"):
        decode_depth(msg, 1000)


def test_result_transform_mpc_and_original_expiry(caplog):
    caplog.set_level("INFO", logger="dimos.path_tracker")
    now = [100.0]
    tracker = Tracker(load_config(), lambda *_: None, clock=lambda: now[0])
    tracker.running = True
    tracker.update_odom((10.1, 2.0, 0.0), "world")
    tracker.update_depth(np.full((10, 10), 3.0))
    result = decode_result(json.dumps(result_payload()))
    assert tracker.submit(result)
    assert not tracker.submit(result)  # no duplicate replanning
    assert tracker.plan_result(tracker.generation, result)
    assert f"path accepted request_stamp_ns={result.stamp_ns}" in caplog.text
    assert (
        f"path rejected request_stamp_ns={result.stamp_ns} reason=stale" in caplog.text
    )
    assert f"MPC plan ready request_stamp_ns={result.stamp_ns}" in caplog.text
    np.testing.assert_allclose(tracker.plan.world_path[:, 0], [10.2, 10.5, 11.0])
    v, w, reason = tracker.command()
    assert v > 0 and abs(w) < 1e-6 and reason == "tracking"
    now[0] += 0.51
    assert tracker.command() == (0, 0, "odom_stale")
    tracker.update_odom((10.1, 2.0, 0), "world")
    now[0] = 102.0
    tracker.update_odom((10.1, 2.0, 0), "world")
    assert tracker.command()[0] < v  # same simple linear decay
    now[0] = 103.0
    tracker.update_odom((10.1, 2.0, 0), "world")
    assert tracker.command() == (0, 0, "expired")


def test_published_velocity_log_uses_active_stamp_and_only_records_state_changes(
    caplog,
):
    caplog.set_level("INFO", logger="dimos.path_tracker")
    sent = []
    sink = VelocityOutput(None, "/cmd_vel", True)
    sink.message_type = lambda: NS(linear=NS(), angular=NS())
    sink.publisher = NS(publish=sent.append)
    tracker = Tracker(load_config(), sink)
    tracker.result = decode_result(json.dumps(result_payload()))

    tracker._emit_command(0.3, 0.1, "tracking")
    tracker._emit_command(0.2, 0.1, "tracking")
    tracker._emit_command(0.0, 0.0, "expired")

    assert len(sent) == 3
    assert [(msg.linear.x, msg.angular.z) for msg in sent] == [
        (0.3, 0.1),
        (0.2, 0.1),
        (0.0, 0.0),
    ]
    events = [record.getMessage() for record in caplog.records]
    assert len(events) == 2
    stamp = tracker.result.stamp_ns
    assert f"request_stamp_ns={stamp} reason=tracking published=True" in events[0]
    assert f"request_stamp_ns={stamp} reason=expired published=True" in events[1]


def test_reference_yaw_and_replaced_planning_job():
    payload = result_payload()
    payload["robot_quat"] = [0, 0, np.sin(np.pi / 4), np.cos(np.pi / 4)]
    result = decode_result(json.dumps(payload))
    np.testing.assert_allclose(result.goal_world, [10, 5], atol=1e-12)
    tracker = Tracker(load_config(), lambda *_: None)
    tracker.running = True
    tracker.update_odom((10, 2, np.pi / 2), "world")
    tracker.update_depth(np.ones((10, 10)))
    tracker.submit(result)
    assert not tracker.plan_result(tracker.generation - 1, result)
    assert tracker.plan_result(tracker.generation, result)
    np.testing.assert_allclose(tracker.plan.world_path[:, 1], [2.2, 2.5, 3], atol=1e-12)


def test_malformed_result_does_not_replace_plan():
    payload = result_payload()
    payload["trajectory"][0][0][0] = float("nan")
    with pytest.raises(ValueError):
        decode_result(json.dumps(payload))
    payload = result_payload()
    payload["trajectory_frame"] = "world"
    with pytest.raises(ValueError):
        decode_result(json.dumps(payload))


def test_collision_keeps_simple_behavior():
    tracker = Tracker(load_config(), lambda *_: None)
    tracker.running = True
    tracker.update_odom((10, 2, 0), "world")
    tracker.update_depth(np.ones((10, 10)))
    result = decode_result(json.dumps(result_payload()))
    tracker.submit(result)
    tracker.plan_result(tracker.generation, result)
    tracker.update_depth(np.full((10, 10), 0.1))
    assert tracker.command() == (0, 0, "collision_depth")
    # Preserve simple: no new depth-age timeout was added in this extraction.
    tracker.update_depth(np.ones((10, 10)))
    assert tracker.command()[2] == "tracking"


def test_solver_nonfinite_path_rejected():
    with pytest.raises(ValueError):
        solve_mpc(np.array([[np.nan, 0, 0], [1, 0, 0]]), 15, 0.1, 0.4, 0.4, 0.2)


def test_clear_plan_stops_and_invalidates_an_older_mpc_job():
    emitted = []
    tracker = Tracker(load_config(), lambda *cmd: emitted.append(cmd))
    tracker.running = True
    tracker.update_odom((10.0, 2.0, 0.0), "world")
    tracker.update_depth(np.full((10, 10), 3.0))
    result = decode_result(json.dumps(result_payload()))
    assert tracker.submit(result)
    assert tracker.plan_result(*tracker.pending)

    payload = result_payload()
    payload["request_stamp_ns"] += 1
    assert tracker.submit(decode_result(json.dumps(payload)))
    pending = tracker.pending
    tracker.clear_plan("task_stopped")

    assert emitted[-1] == (0.0, 0.0, "task_stopped")
    assert tracker.pending is None
    assert not tracker.plan_result(*pending)
    assert tracker.command() == (0.0, 0.0, "no_plan")


def test_motion_tracker_requires_fresh_depth_and_does_not_resume_old_plan():
    now = [100.0]
    tracker = Tracker(
        load_config(), lambda *_: None, clock=lambda: now[0], depth_timeout_s=0.5
    )
    tracker.running = True
    tracker.update_odom((10.0, 2.0, 0.0), "world")
    tracker.update_depth(np.full((10, 10), 3.0))
    payload = result_payload()
    result = decode_result(json.dumps(payload))
    assert tracker.submit(result)
    old_job = tracker.pending
    assert tracker.plan_result(*old_job)

    now[0] += 0.6
    tracker.update_odom((10.1, 2.0, 0.0), "world")
    assert tracker.command() == (0.0, 0.0, "depth_stale")
    payload["request_stamp_ns"] += 1
    next_result = decode_result(json.dumps(payload))
    assert not tracker.submit(next_result)

    tracker.update_depth(np.full((10, 10), 3.0))
    assert tracker.command() == (0.0, 0.0, "no_plan")
    assert not tracker.plan_result(*old_job)
    assert tracker.submit(next_result)
    assert tracker.plan_result(*tracker.pending)
    assert tracker.command()[0] > 0


def test_arrival_uses_fresh_odom_in_target_frame():
    now = [100.0]
    tracker = Tracker(load_config(), lambda *_: None, clock=lambda: now[0])
    tracker.update_odom((10.1, 2.0, 0.0), "world")
    assert tracker.reached((10.0, 2.0), "world")
    assert not tracker.reached((10.0, 2.0), "another_frame")
    assert not tracker.reached((11.0, 2.0), "world")
    now[0] += 0.6
    assert not tracker.reached((10.0, 2.0), "world")


def test_motion_tracker_node_does_not_subscribe_to_unvalidated_results():
    pytest.importorskip("rclpy")
    from dimos.plugins.path_tracker.node import TrackerNode

    subscriptions = []

    class Node:
        def create_subscription(self, message_type, topic, callback, qos):
            subscriptions.append(topic)

    cfg = load_config()
    TrackerNode(Node(), cfg, subscribe_results=False)
    assert cfg["topics"]["result"] not in subscriptions
    assert cfg["topics"]["odom"] in subscriptions
    assert cfg["topics"]["depth"] in subscriptions


def test_motion_tracker_requires_live_rgbd_snapshot():
    ready = [True]
    tracker = Tracker(
        load_config(),
        lambda *_: None,
        depth_timeout_s=0.5,
        inputs_ready=lambda: ready[0],
    )
    tracker.running = True
    tracker.update_odom((10.0, 2.0, 0.0), "world")
    tracker.update_depth(np.full((10, 10), 3.0))
    result = decode_result(json.dumps(result_payload()))
    assert tracker.submit(result)
    assert tracker.plan_result(*tracker.pending)
    ready[0] = False

    assert tracker.command() == (0.0, 0.0, "observation_stale")
    ready[0] = True
    assert tracker.command() == (0.0, 0.0, "no_plan")


def test_rejected_update_invalidates_old_motion_plan_before_next_tick():
    ready = [True]
    tracker = Tracker(
        load_config(), lambda *_: None, depth_timeout_s=0.5,
        inputs_ready=lambda: ready[0],
    )
    tracker.running = True
    tracker.update_odom((10, 2, 0), "world")
    tracker.update_depth(np.ones((10, 10)))
    payload = result_payload()
    result = decode_result(json.dumps(payload))
    assert tracker.submit(result)
    assert tracker.plan_result(tracker.generation, result)
    assert tracker.command()[0] > 0

    ready[0] = False
    payload["request_stamp_ns"] += 1
    assert not tracker.submit(decode_result(json.dumps(payload)))
    ready[0] = True
    assert tracker.command() == (0.0, 0.0, "no_plan")


def test_exploration_lidar_guard_prevents_old_plan_resuming_after_dropout():
    now = [10.0]
    ready = [True]
    tracker = Tracker(load_config(), lambda *_: None, clock=lambda: now[0], depth_timeout_s=0.5)
    tracker.running = True
    tracker.set_motion_guard(lambda: ready[0])
    tracker.update_odom((10, 2, 0), "world")
    tracker.update_depth(np.ones((10, 10)))
    result = decode_result(json.dumps(result_payload()))
    assert tracker.submit(result)
    assert tracker.plan_result(tracker.generation, result)
    assert tracker.command()[0] > 0
    ready[0] = False
    assert tracker.command() == (0, 0, "exploration_lidar_stale")
    ready[0] = True
    assert tracker.command() == (0, 0, "no_plan")


def test_motion_depth_hard_stop_catches_a_close_obstacle_at_low_speed():
    from dimos.plugins.path_tracker.collision import collision_veto

    tracker = Tracker(load_config(), lambda *_: None, depth_timeout_s=0.5)
    depth = np.full((10, 10), 0.10)
    assert collision_veto(tracker.guard, (0, 0, 0), None, None, depth, 0.20, 0) == "depth"


def test_requested_standoff_stops_and_latches_before_waypoint():
    now = [10.0]
    tracker = Tracker(load_config(), lambda *_: None, clock=lambda: now[0])
    tracker.running = True
    tracker.set_standoff((13.0, 2.0), 0.8, "world")
    tracker.update_odom((12.0, 2.0, 0), "world")
    tracker.update_depth(np.ones((10, 10)))
    result = decode_result(json.dumps(result_payload()))
    assert tracker.submit(result)
    assert tracker.plan_result(tracker.generation, result)
    tracker.update_odom((12.25, 2.0, 0), "world")
    assert tracker.command() == (0.0, 0.0, "standoff_reached")
    assert tracker.plan is None
    assert tracker.reached(result.goal_world, "world")
    assert not tracker.set_standoff((14.0, 2.0), 0.8, "world")
    late_payload = result_payload()
    late_payload["request_stamp_ns"] += 1
    assert not tracker.submit(decode_result(json.dumps(late_payload)))
    tracker.clear_standoff()
    assert tracker.set_standoff((14.0, 2.0), 0.8, "world")


@pytest.mark.parametrize("obstacle", [False, True])
def test_sweep_turn_uses_angular_velocity_and_stops(obstacle):
    import math
    import threading

    commands = []
    tracker = Tracker(load_config(), lambda *args: commands.append(args), depth_timeout_s=0.5)
    tracker.running = True
    tracker.update_depth(np.ones((10, 10)))
    tracker.update_odom((0., 0., math.radians(170)), "world")
    tracker.update_scan(np.array([[0.32, 0.]]) if obstacle else np.empty((0, 2)))
    target = math.radians(230)
    if obstacle:
        assert tracker.turn_to(target, "world", lambda: 2) == "collision_lidar"
    else:
        finished = []
        with tracker.cv:
            worker = threading.Thread(target=lambda: finished.append(
                tracker.turn_to(target, "world", lambda: 2)
            ), daemon=True)
            worker.start()
            assert tracker.cv.wait_for(lambda: tracker.turn_goal is not None, timeout=1)
            v, w, reason = tracker.command()
            assert v == 0
            assert w > 0
            assert reason == "exploration_turn"
            tracker.update_odom((0., 0., math.radians(-130)), "world")
        worker.join(timeout=1)
        assert finished == [None]
    assert commands[-1][:2] == (0., 0.)
    assert tracker.command() == (0., 0., "no_plan")


def test_sweep_turn_publishes_zero_on_task_cancellation():
    commands = []
    tracker = Tracker(load_config(), lambda *args: commands.append(args))
    tracker.running = True

    def remaining():
        if tracker.turn_goal is not None:
            raise RuntimeError("cancelled")
        return 2

    with pytest.raises(RuntimeError, match="cancelled"):
        tracker.turn_to(1.0, "world", remaining)
    assert tracker.turn_goal is None
    assert commands[-1][:2] == (0., 0.)
