import json
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_path_tracker import result_payload
from test_xnavdp import header


def capture_events(monkeypatch, module):
    records = []
    monkeypatch.setattr(module, "event", lambda _, name, **fields: records.append((name, fields)),
                        raising=False)
    return records


def test_stop_preserves_active_path_identity_across_task_contexts(monkeypatch):
    from dimos.plugins.path_tracker import tracker as module
    from dimos.plugins.path_tracker.config import load_config
    from dimos.plugins.path_tracker.messages import decode_result
    from dimos.plugins.runtime_log import task_context

    events = capture_events(monkeypatch, module)
    tracker = module.Tracker(load_config(), lambda *_: True)
    tracker.running = True
    tracker.update_odom((10.0, 2.0, 0.0), "world")
    tracker.update_depth(np.full((10, 10), 3.0))
    result = decode_result(json.dumps(result_payload()))
    with task_context("task-a"):
        assert tracker.submit(result)
    generation = tracker.generation
    assert tracker.plan_result(*tracker.pending)
    tracker.clear_plan("task_cancelled")
    record = next(fields for name, fields in events if name == "tracker_velocity")
    assert record["request_stamp_ns"] == result.stamp_ns
    assert record["plan_generation"] == generation
    assert record["task_id"] == "task-a"
    assert (record["linear_x"], record["angular_z"], record["published"]) == (0, 0, True)
    assert record["reason"] == "task_cancelled"


def test_xnavdp_reports_discovery_failure_before_sending_goal(monkeypatch):
    from dimos.plugins.xnavdp import client as module

    events = capture_events(monkeypatch, module)
    client = module.XNavDPClient({})
    goal = NS(header=header(123), pose=NS(position=NS(x=3, y=4)))

    def fail(**_):
        raise TimeoutError("No subscriber on /xnavdp/point_goal")

    client.open_requests = fail
    with pytest.raises(TimeoutError, match="No subscriber"):
        client.infer(goal)
    record = next(fields for name, fields in events if name == "xnavdp_request_finished")
    assert record["request_stamp_ns"] == 123
    assert record["goal_xy"] == [3, 4]
    assert record["stage"] == "discovery"
    assert record["outcome"] == "failed"
    assert record["error_type"] == "TimeoutError"


def test_body_logs_requested_and_forwarded_command_on_transition(monkeypatch):
    from dimos.robot.unitree.go2 import body_bridge as module

    events = capture_events(monkeypatch, module)
    sent = []
    bridge = module.Go2BodyBridge(None, {
        "body": {"allow_motion": True, "max_linear_mps": 0.4, "max_angular_rps": 0.2},
    })
    bridge.connection = NS(move=lambda *args: sent.append(args))
    bridge._on_command(NS(linear=NS(x=1.0, y=0.0), angular=NS(z=0.8)))
    bridge._on_command(NS(linear=NS(x=0.0, y=0.0), angular=NS(z=0.0)))
    records = [fields for name, fields in events if name == "body_command_forwarded"]
    assert sent == [(0.4, 0.0, 0.2), (0.0, 0.0, 0.0)]
    assert [item["source"] for item in records] == ["cmd_vel", "cmd_vel"]
    assert records[0]["received_velocity"] == (1.0, 0.0, 0.8)
    assert records[0]["forwarded_velocity"] == sent[0]
    assert records[-1]["forwarded_velocity"] == sent[-1]
    assert records[-1]["active"] is False


def test_body_stop_estimate_requires_two_low_motion_intervals_after_zero(monkeypatch):
    from dimos.robot.unitree.go2 import body_bridge as module

    events = capture_events(monkeypatch, module)
    monkeypatch.setattr(module, "recording_enabled", lambda: True)
    now = [100.0]
    monkeypatch.setattr(module.time, "monotonic", lambda: now[0])
    bridge = module.Go2BodyBridge(None, {
        "body": {"allow_motion": True, "max_linear_mps": 0.4, "max_angular_rps": 0.2},
    })
    bridge.connection = NS(move=lambda *_: None)
    bridge._on_command(NS(linear=NS(x=0.3, y=0.0), angular=NS(z=0.0)))
    bridge._on_command(NS(linear=NS(x=0.0, y=0.0), angular=NS(z=0.0)))

    pose = NS(header=header(123))
    for index, x in enumerate((0.0, 0.1, 0.101, 0.102), start=1):
        now[0] = 100.0 + index * 0.25
        bridge._sample_motion(pose, [x, 0, 0, 0, 0, 0, 1])
        if index < 4:
            assert not any(name == "body_stop_observed" for name, _ in events)
    record = next(fields for name, fields in events if name == "body_stop_observed")
    assert record["elapsed_ms"] == pytest.approx(1000.0)
    assert record["consecutive_low_motion_samples"] == 2
    assert record["source"] == "odom_pose_delta"
