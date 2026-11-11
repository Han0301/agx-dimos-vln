import json
import threading
from types import SimpleNamespace

import pytest


def outcome():
    return {
        "status": "localized",
        "world_frame": "world",
        "goal_xy": [3.0, 0.0],
        "observation": {"stamp_ns": 100},
        "timing_ms": {"vlm": 10},
    }


def frame(stamp):
    return SimpleNamespace(
        stamp_ns=stamp, odom=SimpleNamespace(header=SimpleNamespace(frame_id="world"))
    )


def model_result(stamp):
    path = [[[0.2, 0.0, 0.0], [0.5, 0.0, 0.0]]]
    return json.dumps(
        dict(
            status="success", request_stamp_ns=stamp, trajectory=path,
            all_trajectory=[path], all_values=[[0.8]], robot_pos=[0.0, 0.0, 0.0],
            robot_quat=[0.0, 0.0, 0.0, 1.0], goal=[3.0, 0.0],
            trajectory_frame="base_link", goal_frame="base_link", world_frame="world",
        )
    )


class Tracker:
    def __init__(self, arrive_after=2):
        self.arrive_after = arrive_after
        self.updates = []
        self.cleared = []

    def reached(self, goal, world):
        assert goal == (3.0, 0.0) and world == "world"
        return len(self.updates) >= self.arrive_after

    def submit(self, result):
        self.updates.append(result.stamp_ns)
        return True

    def clear_plan(self, reason):
        self.cleared.append(reason)


def test_replans_same_world_goal_on_fresh_frames_until_arrival():
    from dimos.plugins.vln_task.approach import ContinuousApproach

    tracker = Tracker()
    requests = []

    def observe(check, *, after_stamp_ns, max_age_s):
        assert max_age_s == 0.5
        return frame(after_stamp_ns + 1)

    def plan(observation, goal, check, timing):
        requests.append((observation.stamp_ns, goal))
        return model_result(observation.stamp_ns), 0.01, 2

    progress = []
    approach = ContinuousApproach(observe, plan, tracker, interval_s=0)
    result = approach(outcome(), lambda: 10, threading.Event(), progress.append)
    assert result["status"] == "arrived"
    assert result["path_updates"] == 2
    assert tracker.updates == [101, 102]
    assert requests == [(101, (3.0, 0.0)), (102, (3.0, 0.0))]
    assert tracker.cleared == ["goal_reached"]
    assert progress[-1]["status"] == "arrived"


def test_stop_clears_path_and_discards_inflight_result():
    from dimos.plugins.vln_task.agent import TaskCancelled
    from dimos.plugins.vln_task.approach import ContinuousApproach

    tracker = Tracker()
    cancel = threading.Event()

    def plan(observation, goal, check, timing):
        if observation.stamp_ns == 102:
            approach.cancel(cancel)
        return model_result(observation.stamp_ns), 0.01, 2

    approach = ContinuousApproach(
        lambda _, **kw: frame(kw["after_stamp_ns"] + 1), plan, tracker, interval_s=0
    )
    with pytest.raises(TaskCancelled):
        approach(outcome(), lambda: 10, cancel)
    assert tracker.updates == [101]
    assert tracker.cleared == ["task_cancelled"]


def test_wrong_stamp_is_discarded_and_next_frame_can_update():
    from dimos.plugins.vln_task.approach import ContinuousApproach

    tracker = Tracker(arrive_after=1)

    def plan(observation, goal, check, timing):
        stamp = 99 if observation.stamp_ns == 101 else observation.stamp_ns
        return model_result(stamp), 0.01, 2

    approach = ContinuousApproach(
        lambda _, **kw: frame(kw["after_stamp_ns"] + 1), plan, tracker, interval_s=0
    )
    result = approach(outcome(), lambda: 10, threading.Event())
    assert result["path_updates"] == 1
    assert tracker.updates == [102]


def test_motion_agent_hands_confirmed_goal_to_loop_and_reports_arrival():
    from dimos.plugins.vln_task.agent import TaskAgent

    calls = []

    class LLM:
        config = {"timeout_s": 30}

        def chat(self, messages, **kwargs):
            assert not calls
            calls.append("route")
            return {"message": {"tool_calls": [{"function": {
                "name": "active_search", "arguments": {"target": "white bin"},
            }}]}}

    def search(target, remaining, progress):
        calls.append(target)
        return outcome()

    def move(detection, remaining, token, progress, *, target):
        assert target == "white bin"
        assert detection["goal_xy"] == [3.0, 0.0]
        calls.append(token)
        return {**detection, "status": "arrived", "path_updates": 2}

    agent = TaskAgent(
        LLM(), None, {"timeout_s": 20, "max_steps": 4},
        navigation_preview=SimpleNamespace(plan=True),
        active_search=search, approach=move,
    )
    cancel = threading.Event()
    result = agent.run("找白色垃圾桶", cancel=cancel)
    assert calls == ["route", "white bin", cancel]
    assert result["navigation"]["status"] == "arrived"
    assert result["navigation"]["path_updates"] == 2


def test_transient_planner_error_keeps_goal_active():
    from dimos.plugins.vln_task.approach import ContinuousApproach
    from dimos.plugins.xnavdp.client import XNavDPInferenceError

    tracker = Tracker(arrive_after=1)
    requests = []

    def plan(observation, goal, check, timing):
        requests.append((observation.stamp_ns, goal))
        if observation.stamp_ns == 101:
            raise XNavDPInferenceError("server lacks that frame")
        return model_result(observation.stamp_ns), 0.01, 2

    approach = ContinuousApproach(
        lambda _, **kw: frame(kw["after_stamp_ns"] + 1), plan, tracker, interval_s=0
    )
    result = approach(outcome(), lambda: 10, threading.Event())
    assert result["status"] == "arrived"
    assert tracker.updates == [102]
    assert requests == [(101, (3.0, 0.0)), (102, (3.0, 0.0))]


@pytest.mark.parametrize(
    ("status", "target_x", "expected_goal_x"),
    [("localized", 3.8, 3.2), ("not_visible", None, 3.0), ("localized", 5.0, 3.0)],
)
def test_vlm_refresh_runs_during_planning_and_updates_next_point_goal(
    status, target_x, expected_goal_x,
):
    from dimos.plugins.vln_task.approach import ContinuousApproach

    initial = {
        "status": "localized",
        "world_frame": "world",
        "target_xyz": [3.6, 0.0, 0.0],
        "goal_xy": [3.0, 0.0],
        "observation": {"stamp_ns": 100},
        "timing_ms": {},
    }
    first_plan_started = threading.Event()
    refresh_started = threading.Event()
    accepted = []
    requests = []
    tracker = SimpleNamespace(
        reached=lambda *_: len(accepted) >= 2,
        submit=lambda result: accepted.append(result.stamp_ns) or True,
        clear_plan=lambda _: None,
    )

    def observe(check, *, after_stamp_ns, max_age_s):
        stamp = after_stamp_ns + 1
        return SimpleNamespace(
            stamp_ns=stamp,
            odom=SimpleNamespace(header=SimpleNamespace(frame_id="world")),
        )

    def detect(frame, target, remaining):
        assert target == "white bin"
        assert remaining() > 0
        assert first_plan_started.wait(1)
        refresh_started.set()
        if status != "localized":
            return {"status": status}
        return {
            **initial,
            "goal_xy": [3.2, 0.0],
            "target_xyz": [target_x, 0.0, 0.0],
            "observation": {"stamp_ns": frame.stamp_ns},
        }

    def plan(frame, goal, check, timing):
        requests.append((frame.stamp_ns, goal))
        if len(requests) == 1:
            first_plan_started.set()
            assert refresh_started.wait(1), "VLM should run while XNavDP is planning"
        path = [[[0.2, 0.0, 0.0], [0.5, 0.0, 0.0]]]
        return json.dumps(dict(
            status="success", request_stamp_ns=frame.stamp_ns, trajectory=path,
            all_trajectory=[path], all_values=[[0.8]], robot_pos=[0.0, 0.0, 0.0],
            robot_quat=[0.0, 0.0, 0.0, 1.0], goal=goal,
            trajectory_frame="base_link", goal_frame="base_link", world_frame="world",
        )), 0.01, 2

    approach = ContinuousApproach(
        observe, plan, tracker, detect=detect, refresh_interval_s=0.01, interval_s=0.05,
        max_target_shift_m=0.55,
    )
    try:
        result = approach(initial, lambda: 10, threading.Event(), target="white bin")
    finally:
        approach.close()
    assert result["status"] == "arrived"
    assert result["goal_xy"] == [expected_goal_x, 0.0]
    assert [goal for _, goal in requests] == [(3.0, 0.0), (expected_goal_x, 0.0)]
    assert requests[1][0] > requests[0][0]
