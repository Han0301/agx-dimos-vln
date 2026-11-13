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
        details = {}
        assert tracker.command(collision_details=details) == (0.0, 0.1, "rotate_only")
        assert tracker.command() == (0.0, 0.1, "rotate_only")

    veto, = records(run, "path_tracker", "tracker_collision_veto")
    assert veto["reason"] == "map"
    assert veto["action"] == "rotate_only"
    assert veto["blocking_cell_xy"] == [29, 22]
    assert veto["costmap_stamp_ns"] == 12
    assert (run / veto["costmap_path"]).read_bytes() == data.tobytes()
    assert details["costmap_max_checked_cost"] == 73
    assert details["rotation_check"]["costmap_max_checked_cost"] == 0


def test_tracker_records_pass_veto_and_published_recovery(tmp_path):
    config = load_config()
    config["collision"]["cost_threshold"] = 80
    tracker = Tracker(config, lambda *_: True, clock=lambda: 10.0)
    tracker.running = True
    tracker.update_odom((0.0, 0.0, 0.0), "world", stamp_ns=10)
    tracker.update_depth(np.ones((10, 10)), stamp_ns=11)
    tracker.plan = SimpleNamespace(
        command=lambda *_: (0.4, 0.0), started=10.0, commands=[(0.4, 0.0)], dt=0.1,
    )
    origin = SimpleNamespace(position=SimpleNamespace(x=-1.0, y=-1.0))
    info = SimpleNamespace(width=40, height=40, resolution=0.05, origin=origin)

    with capture("web_input", base=tmp_path) as run:
        for stamp, cost, request in ((12, 79, 100), (13, 80, 100), (14, 79, 101)):
            if request == 101:
                tracker._emit_command(0.0, 0.0, "expired", request_stamp_ns=100)
            data = np.zeros((40, 40), dtype=np.int8)
            data[22, 29] = cost
            tracker.update_costmap(
                OccupancyGrid(SimpleNamespace(info=info, data=data.ravel())), stamp_ns=stamp,
            )
            details = {}
            tracker._emit_command(
                *tracker.command(collision_details=details),
                request_stamp_ns=request, collision_details=details,
            )

    first, blocked, expired, resumed = records(run, "path_tracker", "tracker_command_sample")
    for sample, cost, stamp in ((first, 79, 12), (blocked, 80, 13), (resumed, 79, 14)):
        assert sample["collision_check"]["cost_threshold"] == 80
        assert sample["collision_check"]["costmap_max_checked_cost"] == cost
        assert sample["costmap_stamp_ns"] == stamp
        assert sample["published"] is True
    assert [first["reason"], blocked["reason"], resumed["reason"]] == [
        "tracking", "collision_map", "tracking",
    ]
    assert blocked["linear_x"] == 0.0
    assert expired["reason"] == "expired"
    assert "collision_check" not in expired
    assert resumed["linear_x"] == 0.4
    recovery, = records(run, "path_tracker", "tracker_collision_recovered")
    assert recovery["previous_reason"] == "collision_map"
    assert recovery["previous_request_stamp_ns"] == 100
    assert recovery["request_stamp_ns"] == 101
    assert recovery["costmap_stamp_ns"] == 14


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
