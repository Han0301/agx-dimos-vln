import json
from types import SimpleNamespace

import numpy as np
import pytest

from dimos.plugins.path_tracker.collision import collision_veto
from dimos.plugins.path_tracker.config import load_config
from dimos.plugins.path_tracker.messages import OccupancyGrid
from dimos.plugins.path_tracker.tracker import Tracker
from dimos.plugins.runtime_log import capture
from dimos.vln_task.search.frontier import FrontierPlanner


def records(run, name, kind):
    return [
        record for line in (run / f"modules/{name}.jsonl").read_text().splitlines()
        if (record := json.loads(line)).get("event") == kind
    ]


def test_frontier_logs_why_search_cannot_start(tmp_path):
    grid = np.zeros((20, 20), dtype=np.int8)
    grid[10, 10] = -1
    with capture("web_input", base=tmp_path) as run:
        assert FrontierPlanner().select(
            grid, origin_xy=(0, 0), resolution=0.1, robot_xy=(1.05, 1.05),
        ) is None

    outcome, = records(run, "active_search", "frontier_selection")
    assert outcome["reason"] == "start_unknown"
    assert outcome["start_cell_xy"] == [10, 10]
    assert outcome["start_cost"] == -1


def test_frontier_reports_rejected_candidates(tmp_path):
    grid = np.full((30, 30), -1, dtype=np.int8)
    grid[5:25, 5:25] = 0
    with capture("web_input", base=tmp_path) as run:
        assert FrontierPlanner().select(
            grid, origin_xy=(0, 0), resolution=0.1, robot_xy=(1.55, 1.55),
            allow_frontier=lambda _: False,
        ) is None

    outcome, = records(run, "active_search", "frontier_selection")
    assert outcome["reason"] == "all_reachable_candidates_rejected"
    assert outcome["rejections"]["filtered_frontier"] > 0
    assert outcome["reachable_candidate_cells"] == outcome["rejections"]["filtered_frontier"]


def test_costmap_reports_duplicate_source_stamp_without_refreshing_age(tmp_path):
    now = [10.0]
    tracker = Tracker(load_config(), lambda *_: None, clock=lambda: now[0])
    with capture("web_input", base=tmp_path) as run:
        tracker.update_costmap(object(), stamp_ns=200)
        now[0] = 12.0
        tracker.update_costmap(object(), stamp_ns=200)
        tracker.update_costmap(object(), stamp_ns=200)

    rejected, = records(run, "path_tracker", "tracker_costmap_rejected")
    assert rejected["reason"] == "duplicate_stamp"
    assert rejected["incoming_costmap_stamp_ns"] == 200
    assert rejected["costmap_stamp_ns"] == 200
    assert rejected["costmap_age_ms"] == 2000
    assert tracker.costmap_received == 10.0


def test_collision_veto_reports_exact_blocking_cell():
    data = np.zeros((40, 40), dtype=np.int8)
    data[22, 29] = 73
    origin = SimpleNamespace(position=SimpleNamespace(x=-1.0, y=-1.0))
    info = SimpleNamespace(width=40, height=40, resolution=0.05, origin=origin)
    costmap = OccupancyGrid(SimpleNamespace(info=info, data=data.ravel()))
    tracker = Tracker(load_config(), lambda *_: None)
    details = {}

    assert collision_veto(
        tracker.guard, (0.0, 0.0, 0.0), costmap, None, np.ones((10, 10)),
        0.4, 0.0, details=details,
    ) == "map"
    assert details["blocking_cell_xy"] == (29, 22)
    assert details["blocking_cost"] == 73
    assert details["cost_threshold"] == 50
    assert details["sweep_step"] > 0


def test_collision_veto_reports_exact_blocking_depth_sample():
    tracker = Tracker(load_config(), lambda *_: None)
    depth = np.full((10, 10), 2.0)
    depth[3, 4] = 0.1
    details = {}
    assert collision_veto(
        tracker.guard, (0.0, 0.0, 0.0), None, None, depth,
        0.6, 0.0, details=details,
    ) == "depth"
    assert details["blocking_pixel_xy"] == (4, 3)
    assert details["min_depth_m"] == pytest.approx(0.1)
    assert details["hard_stop_below_m"] == pytest.approx(tracker.guard.depth_obstacle_m / 2)


def test_tracker_veto_archives_exact_costmap_and_rotation_fallback(tmp_path):
    data = np.zeros((40, 40), dtype=np.int8)
    data[22, 29] = 73
    origin = SimpleNamespace(position=SimpleNamespace(x=-1.0, y=-1.0))
    info = SimpleNamespace(width=40, height=40, resolution=0.05, origin=origin)
    costmap = OccupancyGrid(SimpleNamespace(info=info, data=data.ravel()))
    tracker = Tracker(load_config(), lambda *_: None, clock=lambda: 10.0)
    tracker.running = True
    tracker.update_odom((0.0, 0.0, 0.0), "world", stamp_ns=10)
    tracker.update_depth(np.ones((10, 10)), stamp_ns=11)
    tracker.update_costmap(costmap, stamp_ns=12)
    tracker.plan = SimpleNamespace(
        command=lambda *_: (0.4, 0.1), started=10.0, commands=[(0.4, 0.1)], dt=0.1,
    )

    with capture("web_input", base=tmp_path) as run:
        assert tracker.command() == (0.0, 0.1, "rotate_only")
        assert tracker.command() == (0.0, 0.1, "rotate_only")

    veto, = records(run, "path_tracker", "tracker_collision_veto")
    assert veto["reason"] == "map"
    assert veto["action"] == "rotate_only"
    assert veto["blocking_cell_xy"] == [29, 22]
    assert veto["costmap_stamp_ns"] == 12
    assert (run / veto["costmap_path"]).read_bytes() == data.tobytes()


def test_lidar_costmap_logs_missing_odom_and_frame_mismatch(tmp_path):
    pytest.importorskip("nav_msgs.msg")
    pytest.importorskip("sensor_msgs.msg")
    from dimos.vln_task.costmap import LidarCostmap

    tracker = Tracker(load_config(), lambda *_: None)
    node = SimpleNamespace(create_publisher=lambda *_: None, create_subscription=lambda *_: None)
    costmap = LidarCostmap(node, tracker, "/lidar", "/global_costmap")
    msg = SimpleNamespace(header=SimpleNamespace(
        frame_id="lidar", stamp=SimpleNamespace(sec=1, nanosec=12),
    ))

    with capture("web_input", base=tmp_path) as run:
        costmap._receive(msg)
        costmap._receive(msg)
        tracker.update_odom((0.0, 0.0, 0.0), "world", stamp_ns=99)
        costmap._receive(msg)

    missing, mismatch = records(run, "vln_task", "lidar_costmap_update")
    assert missing["reason"] == "missing_odom"
    assert mismatch["reason"] == "frame_mismatch"
    assert mismatch["lidar_stamp_ns"] == 1_000_000_012
    assert mismatch["odom_stamp_ns"] == 99


def test_costmap_guard_does_not_lock_tracker_for_disabled_logging(monkeypatch):
    from dimos.plugins.path_tracker import node

    monkeypatch.setattr(node, "recording_enabled", lambda: False, raising=False)

    class TrackerWithoutDiagnosticLock:
        @property
        def cv(self):
            pytest.fail("disabled logging took the tracker diagnostic lock")

    def reject(_):
        raise ValueError("invalid costmap")

    adapter = object.__new__(node.TrackerNode)
    adapter.tracker = TrackerWithoutDiagnosticLock()
    adapter.last_error = {}
    adapter._guard("costmap", reject)(None)

    assert "costmap" in adapter.last_error
