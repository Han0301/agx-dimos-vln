import json
import threading
from types import SimpleNamespace as NS

import numpy as np
import pytest

from dimos.plugins.runtime_log import capture

from test_navigation_safety_logging import records


def frame(stamp, x):
    pose = NS(position=NS(x=x, y=0.0, z=0.0), orientation=NS(x=0., y=0., z=0., w=1.))
    return NS(stamp_ns=stamp, odom=NS(header=NS(frame_id="world"), pose=pose))


class Mapper:
    def update(self, obs):
        self.stamp = obs.stamp_ns
        return True

    def snapshot(self, *, frame, require_cloud=False):
        return NS(
            frame=frame, stamp_ns=self.stamp, grid=np.zeros((3, 3)),
            origin_xy=(0., 0.), resolution=0.1,
        )


class Planner:
    def path_is_clear(self, grid, *, origin_xy, resolution, robot_xy, path_xy):
        return np.max(path_xy[:, 0]) < 1.5


@pytest.mark.parametrize("status", ["localized", "invalid_depth", "not_visible", "invalid_detection"])
def test_current_frame_vlm_branch_precedes_independent_exploration(status):
    from dimos.vln_task.find_object import FindObject

    first = frame(100, 0)
    events = []
    progress = []

    def detect(observation, target, remaining, **kwargs):
        assert observation is first
        events.append("vlm")
        return {"status": status}

    def explore(target, remaining, report, cancel, first_view, initial_detection):
        assert first_view is first
        assert initial_detection["status"] == status
        events.append("explore")
        return {"status": "search_exhausted"}

    def localized(observation, detection, remaining):
        events.append("localized")
        return detection

    find = FindObject(lambda _: first, detect, explore, localized)
    result = find("white bin", lambda: 10, progress.append, threading.Event())
    assert progress[0]["status"] == "checking_current_view"
    if status != "localized":
        assert events == ["vlm", "explore"]
        assert result["status"] == "search_exhausted"
    else:
        assert events == ["vlm", "localized"]
        assert result["status"] == status


@pytest.mark.parametrize("x, yaw_deg, expected_stamp, motion_status", [
    (0.15, 0, 106, None), (0, 10, 106, None), (0, 0, 106, None),
    (0, 0, None, None), (0.15, 0, 106, "arrived"),
])
def test_target_direction_rechecks_stopped_view_without_pose_threshold(
    monkeypatch, tmp_path, x, yaw_deg, expected_stamp, motion_status,
):
    from dimos.plugins.active_search import exploration

    now = [0.]
    monkeypatch.setattr(exploration.time, "monotonic", lambda: now[0])
    first = frame(100, 0.)
    first.depth = np.zeros((100, 100))
    first.k = [100., 0., 50., 0., 100., 50., 0., 0., 1.]
    first.distortion = []
    stopped = False

    def observe(check, **kwargs):
        check()
        if "after_stamp_ns" in kwargs:
            assert stopped
            assert kwargs["after_stamp_ns"] == 105
            if expected_stamp is None:
                raise TimeoutError("no stopped frame")
            view = frame(106, x)
            view.observation_id = "stopped-view"
            half_yaw = np.deg2rad(yaw_deg) / 2
            view.odom.pose.orientation.z = np.sin(half_yaw)
            view.odom.pose.orientation.w = np.cos(half_yaw)
            return view
        return frame(101, 0.)

    def approach(outcome, remaining, cancel, on_progress, *, motion_guard):
        nonlocal stopped
        x, y = outcome["goal_xy"]
        assert x > 0 and y < 0
        assert remaining() == pytest.approx(7.)
        now[0] = 2.
        on_progress({"status": "tracking"})
        assert remaining() == pytest.approx(6.)
        if motion_status is not None:
            stopped = True
            return {"status": motion_status}
        now[0] += 1.
        on_progress({"status": "tracking"})
        assert remaining() == pytest.approx(5.)
        now[0] += 5.
        try:
            remaining()
        finally:
            stopped = True

    step = exploration.DirectedExploration(
        observe, lambda: 105, approach, Mapper(), Planner(),
        threading.Event(), settle_s=0,
    )
    detection = {"bbox_2d": [700, 400, 900, 600]}
    with capture("web_input", base=tmp_path) as run:
        if expected_stamp is None:
            with pytest.raises(TimeoutError, match="no stopped frame"):
                step.toward_target(first, detection, lambda: 10, {"pitch": 0.})
        else:
            view = step.toward_target(first, detection, lambda: 10, {"pitch": 0.})
            assert view.stamp_ns == expected_stamp

    outcome, = records(run, "active_search", "exploration_leg_finished")
    assert outcome["leg_kind"] == "directed"
    assert outcome["phase"] == "stopped_observation"
    if expected_stamp is None:
        assert outcome["reason"] == "no_new_view"
    else:
        assert outcome["reason"] == "new_view"
        assert outcome["observation_id"] == "stopped-view"
        assert outcome["motion_reason"] == (motion_status or "motion_window_elapsed")


@pytest.mark.parametrize("detour", [True, False])
def test_directed_approach_submits_path_to_tracker(detour, monkeypatch, tmp_path):
    from dimos.vln_task.approach import ContinuousApproach
    from dimos.vln_task.search import exploration
    from dimos.vln_task.search.frontier import FrontierPlanner

    now = [0.0]
    monkeypatch.setattr(exploration.time, "monotonic", lambda: now[0])
    origin = np.array([-1.05, -1.05])
    grid = np.zeros((30, 30), dtype=np.int8)
    grid[13, 13] = 100
    planner = FrontierPlanner()
    assert all(not planner.path_is_clear(
        grid, origin_xy=origin, resolution=0.1, robot_xy=(0.0, 0.0),
        path_xy=[(distance, 0.0)],
    ) for distance in (0.60, 0.45, 0.30))

    class SceneMapper(Mapper):
        def snapshot(self, *, frame, require_cloud=False):
            return NS(frame=frame, stamp_ns=self.stamp, grid=grid, origin_xy=origin, resolution=0.1)

    class Tracker:
        def __init__(self):
            self.submitted = []

        def reached(self, goal, world_frame):
            return bool(self.submitted)

        def submit(self, result):
            self.submitted.append(result)
            return True

        def clear_plan(self, reason):
            assert reason == "goal_reached"

    requests = []
    tracker = Tracker()

    def plan(observation, goal, check, timing):
        requests.append(goal)
        assert goal == pytest.approx((2.0, 0.0))
        points = [[0.0, -0.25, 0.0], [2.0, -0.25, 0.0]] if detour else [[0.30, 0.0, 0.0]]
        path = [[*points, [2.0, 0.0, 0.0]]]
        return json.dumps({
            "status": "success", "request_stamp_ns": observation.stamp_ns,
            "trajectory": path, "all_trajectory": [path], "all_values": [[0.8]],
            "robot_pos": [0.0, 0.0, 0.0], "robot_quat": [0.0, 0.0, 0.0, 1.0],
            "goal": goal, "trajectory_frame": "base_link", "goal_frame": "base_link",
            "world_frame": "world",
        }), 0.01, len(path[0])

    def observe(check, **kwargs):
        if "received_after" in kwargs:
            assert tracker.submitted
            return frame(104, 2.0)
        view = frame(102 if "after_stamp_ns" in kwargs else (103 if requests else 101), 0.0)
        view.received_at = now[0]
        return view

    first = frame(100, 0.)
    first.depth = np.zeros((100, 100))
    first.k = [100., 0., 50., 0., 100., 50., 0., 0., 1.]
    first.distortion = []
    approach = ContinuousApproach(observe, plan, tracker, interval_s=0)
    step = exploration.DirectedExploration(
        observe, lambda: 103, approach, SceneMapper(), planner, threading.Event(),
    )
    with capture("web_input", base=tmp_path) as run:
        view = step.toward_target(
            first, {"bbox_2d": [400, 400, 600, 600]}, lambda: 10, {"pitch": 0.},
        )

    assert len(requests) == 1
    goal, = records(run, "active_search", "exploration_goal_selected")
    assert goal["distance_m"] == 2.0
    assert len(tracker.submitted) == 1
    assert view.stamp_ns == 104
