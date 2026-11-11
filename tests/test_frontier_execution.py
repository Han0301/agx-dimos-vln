import threading
from types import SimpleNamespace as NS

import numpy as np
import pytest


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
    from dimos.plugins.vln_task.find_object import FindObject

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
    if status in {"not_visible", "invalid_depth"}:
        assert events == ["vlm", "explore"]
        assert result["status"] == "search_exhausted"
    else:
        assert events == (["vlm", "localized"] if status == "localized" else ["vlm"])
        assert result["status"] == status


@pytest.mark.parametrize("x, yaw_deg, expected_stamp", [(0.15, 0, 106), (0, 10, 106), (0, 0, 106)])
def test_target_direction_rechecks_stopped_view_without_pose_threshold(monkeypatch, x, yaw_deg, expected_stamp):
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
            view = frame(106, x)
            half_yaw = np.deg2rad(yaw_deg) / 2
            view.odom.pose.orientation.z = np.sin(half_yaw)
            view.odom.pose.orientation.w = np.cos(half_yaw)
            return view
        return frame(101, 0.)

    def approach(outcome, remaining, cancel, on_progress, *, accept_path, motion_guard):
        nonlocal stopped
        x, y = outcome["goal_xy"]
        assert x > 0 and y < 0
        now[0] = 2.
        on_progress({"status": "tracking"})
        assert remaining() == pytest.approx(2.)
        now[0] += 2.
        try:
            remaining()
        finally:
            stopped = True

    step = exploration.DirectedExploration(
        observe, lambda: 105, approach, Mapper(), Planner(),
        threading.Event(), settle_s=0,
    )
    view = step.toward_target(
        first, {"bbox_2d": [700, 400, 900, 600]}, lambda: 10, {"pitch": 0.},
    )
    assert getattr(view, "stamp_ns", None) == expected_stamp


def test_blocked_target_direction_returns_without_motion():
    from dimos.plugins.active_search.exploration import DirectedExploration

    class BlockedPlanner(Planner):
        def path_is_clear(self, grid, **kwargs):
            return False

    def must_not_approach(*args, **kwargs):
        pytest.fail("Motion requested toward a blocked target direction")

    first = frame(100, 0.)
    first.depth = np.zeros((100, 100))
    first.k = [100., 0., 50., 0., 100., 50., 0., 0., 1.]
    first.distortion = []
    step = DirectedExploration(
        lambda *args, **kwargs: frame(101, 0.), lambda: 101,
        must_not_approach, Mapper(), BlockedPlanner(), threading.Event(),
    )
    assert step.toward_target(
        first, {"bbox_2d": [400, 400, 600, 600]}, lambda: 10, {"pitch": 0.},
    ) is None
