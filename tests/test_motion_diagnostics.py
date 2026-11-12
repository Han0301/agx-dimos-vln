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
    monkeypatch.setattr(module, "recording_enabled", lambda: True)
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
    monkeypatch.setattr(module, "recording_enabled", lambda: True)
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
    monkeypatch.setattr(module, "recording_enabled", lambda: True)
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


@pytest.mark.parametrize("inline", [True, False])
def test_xnavdp_replay_records_native_poses_and_candidates(monkeypatch, tmp_path, inline):
    from dimos.plugins.runtime_log import capture, task_context
    from dimos.plugins.xnavdp import client as module

    monkeypatch.setattr(module, "_INLINE_REPLAY_BYTES", 65536 if inline else 0)
    result = result_payload()
    stamp = result["request_stamp_ns"]
    native_header = header(stamp, "base_link")
    native = NS(
        header=native_header,
        poses=[NS(header=native_header, pose=NS(
            position=NS(x=x, y=y, z=z), orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0),
        )) for x, y, z in result["trajectory"][0]],
    )
    client = module.XNavDPClient({"response_timeout_s": 0.1})
    client.open_requests = lambda **_: None
    client.prepare = lambda *_, **__: {"release": "release-a", "transport": "transport-a"}

    def reply(_):
        client._receive("path", native, native_header)
        client._result(NS(data=json.dumps(result)))

    client.goal_pub = NS(publish=reply)
    goal = NS(header=header(stamp), pose=NS(position=NS(x=13.0, y=2.0)))
    with capture("web_input", base=tmp_path) as run, task_context("task-a"):
        parts, _ = client.infer(goal)
    assert parts["path"] is native
    records = [json.loads(line) for line in (run / "modules/xnavdp.jsonl").read_text().splitlines()]
    replay = next(fields for fields in records if fields.get("event") == "xnavdp_trajectory")
    assert replay["request_stamp_ns"] == stamp
    assert replay["task_id"] == "task-a"
    final = next(fields for fields in records if fields.get("event") == "xnavdp_request_finished")
    assert final["server_release"] == "release-a"
    assert final["server_transport"] == "transport-a"
    if not inline:
        assert replay["artifact_outcome"] == "queued"
        replay = json.loads((run / replay["artifact_path"]).read_text())
    assert replay["result"]["all_trajectory"] == [[[[0.2, 0, 0], [0.5, 0, 0], [1, 0, 0]]]]
    assert replay["result"]["all_values"] == [[1.0]]
    assert replay["native_path"]["poses"][1] == {
        "position_xyz": [0.5, 0.0, 0.0], "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
    }


def test_tracker_replay_has_full_mpc_and_repeated_command_samples(monkeypatch):
    from dimos.plugins.path_tracker import tracker as module
    from dimos.plugins.path_tracker.config import load_config
    from dimos.plugins.path_tracker.messages import decode_result
    from dimos.plugins.runtime_log import task_context

    events = capture_events(monkeypatch, module)
    monkeypatch.setattr(module, "recording_enabled", lambda: True, raising=False)
    issued = []
    tracker = module.Tracker(load_config(), lambda *args: issued.append(args) or True)
    tracker.running = True
    tracker.update_odom((10.0, 2.0, 0.0), "world")
    tracker.update_depth(np.full((10, 10), 3.0))
    result = decode_result(json.dumps(result_payload()))
    with task_context("task-a"):
        assert tracker.submit(result)
    assert tracker.plan_result(*tracker.pending)
    tracker._emit_command(0.30, 0.05, "tracking")
    tracker._emit_command(0.25, 0.05, "tracking")

    ready = next(fields for name, fields in events if name == "tracker_mpc_ready")
    assert ready["world_path"] == [[10.2, 2.0, 0.0], [10.5, 2.0, 0.0], [11.0, 2.0, 0.0]]
    assert len(ready["commands"]) == tracker.cfg["mpc_horizon"]
    np.testing.assert_allclose(ready["commands"], tracker.plan.commands)
    samples = [fields for name, fields in events if name == "tracker_command_sample"]
    assert [sample["linear_x"] for sample in samples] == [0.30, 0.25]
    assert all(sample["task_id"] == "task-a" and sample["published"] for sample in samples)
    assert all(sample["request_stamp_ns"] == result.stamp_ns for sample in samples)
    assert samples[0]["issued_monotonic_ns"] <= samples[1]["issued_monotonic_ns"]


def test_turn_trace_crosses_threads_and_includes_stop(monkeypatch):
    import threading

    from dimos.plugins.path_tracker import tracker as module
    from dimos.plugins.path_tracker.config import load_config
    from dimos.plugins.runtime_log import task_context

    events = capture_events(monkeypatch, module)
    monkeypatch.setattr(module, "recording_enabled", lambda: True, raising=False)
    tracker = module.Tracker(load_config(), lambda *_: True)
    tracker.running = True
    tracker.update_odom((0.0, 0.0, 0.0), "world")
    tracker.update_depth(np.full((10, 10), 3.0))
    tracker.update_scan(np.empty((0, 2)))
    result = []

    def turn():
        with task_context("turn-task"):
            result.append(tracker.turn_to(1.0, "world", lambda: 2))

    with tracker.cv:
        worker = threading.Thread(target=turn, daemon=True)
        worker.start()
        try:
            assert tracker.cv.wait_for(lambda: tracker.turn_goal is not None, timeout=1)
            tracker._emit_command(*tracker.command())
        finally:
            tracker.update_odom((0.0, 0.0, 1.0), "world")
    worker.join(timeout=1)
    assert result == [None]
    samples = [fields for name, fields in events if name == "tracker_command_sample"]
    assert [item["reason"] for item in samples] == ["exploration_turn", "exploration_turn_stopped"]
    assert all(item["task_id"] == "turn-task" and item["request_stamp_ns"] is None for item in samples)
    assert all(item["turn_generation"] == samples[0]["turn_generation"] for item in samples)


def test_body_replay_keeps_consecutive_forwarded_commands(monkeypatch):
    from dimos.robot.unitree.go2 import body_bridge as module

    events = capture_events(monkeypatch, module)
    monkeypatch.setattr(module, "recording_enabled", lambda: True)
    bridge = module.Go2BodyBridge(None, {
        "body": {"allow_motion": True, "max_linear_mps": 0.4, "max_angular_rps": 0.2},
    })
    bridge.connection = NS(move=lambda *_: None)
    bridge._on_command(NS(linear=NS(x=0.9, y=0.0), angular=NS(z=0.1)))
    bridge._on_command(NS(linear=NS(x=0.2, y=0.0), angular=NS(z=0.1)))
    bridge._zero("watchdog")
    samples = [fields for name, fields in events if name == "body_command_sample"]
    assert [item["forwarded_velocity"] for item in samples] == [
        (0.4, 0.0, 0.1), (0.2, 0.0, 0.1), (0.0, 0.0, 0.0),
    ]
    assert samples[0]["received_velocity"] == (0.9, 0.0, 0.1)
    assert samples[-1]["source"] == "watchdog"
    assert all(item["received_monotonic_ns"] <= item["forwarded_monotonic_ns"] for item in samples)


def test_tracker_without_recording_does_not_construct_control_trace(monkeypatch):
    from dimos.plugins.path_tracker import tracker as module
    from dimos.plugins.path_tracker.config import load_config
    from dimos.plugins.path_tracker.messages import decode_result

    def unexpected(*args, **kwargs):
        pytest.fail("unrecorded motion constructed diagnostic data")

    monkeypatch.setattr(module, "recording_enabled", lambda: False)
    for name in ("_PathTrace", "_TurnTrace", "task_context", "event"):
        monkeypatch.setattr(module, name, unexpected)
    monkeypatch.setattr(module.Tracker, "_input_fields", unexpected)

    issued = []
    tracker = module.Tracker(
        load_config(), lambda *args: issued.append(args) or True, inputs_ready=lambda: True,
    )
    tracker.running = True
    tracker.update_odom((10.0, 2.0, 0.0), "world")
    tracker.update_depth(np.full((10, 10), 3.0))
    tracker.update_scan(np.empty((0, 2)))
    assert tracker.submit(decode_result(json.dumps(result_payload())))
    assert tracker.plan_result(*tracker.pending)
    tracker._emit_command(0.2, 0.1, "tracking")
    assert tracker.turn_to(0.0, "world", lambda: 1.0) is None
    assert issued == [(0.2, 0.1, "tracking"), (0.0, 0.0, "exploration_turn_stopped")]


def test_body_without_recording_only_reads_watchdog_timestamp(monkeypatch):
    from dimos.robot.unitree.go2 import body_bridge as module

    monkeypatch.setattr(module, "recording_enabled", lambda: False)
    events = capture_events(monkeypatch, module)
    clock_reads = []
    monkeypatch.setattr(module, "time", NS(monotonic=lambda: clock_reads.append(True) or 100.0))
    forwarded = []
    bridge = module.Go2BodyBridge(None, {
        "body": {"allow_motion": True, "max_linear_mps": 0.4, "max_angular_rps": 0.2},
    })
    bridge.connection = NS(move=lambda *args: forwarded.append(args))
    bridge._on_command(NS(linear=NS(x=0.9, y=0.0), angular=NS(z=0.1)))
    bridge._zero("watchdog")

    assert forwarded == [(0.4, 0.0, 0.1), (0.0, 0.0, 0.0)]
    assert bridge.last_command == 100.0
    assert clock_reads == [True]
    assert not events


def test_xnavdp_without_recording_still_pairs_results_and_returns_requested_timings(monkeypatch):
    from dimos.plugins.xnavdp import client as module

    monkeypatch.setattr(module, "recording_enabled", lambda: False)
    events = capture_events(monkeypatch, module)
    result = result_payload()
    stamp = result["request_stamp_ns"]
    native_header = header(stamp, "base_link")
    native = NS(
        header=native_header,
        poses=[NS(header=native_header, pose=NS(position=NS(x=x, y=y, z=z)))
               for x, y, z in result["trajectory"][0]],
    )
    client = module.XNavDPClient({"response_timeout_s": 0.1})
    client.open_requests = lambda **_: None
    client.prepare = lambda *_, **__: None
    payload = NS(data=json.dumps(result))

    def reply(_):
        client._receive("path", native, header(stamp + 1, "base_link"))
        client._result(NS(data="invalid json"))
        client._receive("path", native, native_header)
        client._result(payload)
        assert client.part_arrivals is None
        assert client.rejected_parts is None

    client.goal_pub = NS(publish=reply)
    goal = NS(header=header(stamp), pose=NS(position=NS(x=13.0, y=2.0)))
    timing = {}
    parts, elapsed = client.infer(goal, timing_ms=timing)

    assert parts == {"path": native, "result": payload}
    assert elapsed >= 0
    assert {"xnavdp_discovery", "xnavdp_health", "xnavdp_response", "xnavdp_validation"} <= timing.keys()
    assert not events
