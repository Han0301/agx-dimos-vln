from pathlib import Path

import numpy as np

from dimos.plugins.path_tracker.config import load_config
from dimos.plugins.path_tracker.tracker import Tracker

ROOT = Path(__file__).resolve().parents[1]


def test_vln_motion_uses_lidar_via_costmap():
    config = load_config(ROOT / "config/plugins/path_tracker/tracker_motion.yaml")
    assert config["topics"]["lidar"] is None
    assert config["topics"]["costmap"] == "/global_costmap"
    assert config["costmap_lidar"] == "/lidar"


def test_voxel_height_change_is_an_obstacle():
    from dimos.vln_task.costmap import lidar_height_costs

    x, y = np.meshgrid(np.arange(-10, 11) * 0.1 + 0.05, np.arange(-10, 11) * 0.1 + 0.05)
    z = np.where(x >= 0.4, 0.4, 0.0)
    costs, origin = lidar_height_costs(np.column_stack((x.ravel(), y.ravel(), z.ravel())), (0, 0))
    assert costs[0, 0] == -1
    gx, gy = np.floor((np.array([0.40, 0.05]) - origin) / 0.05).astype(int)
    assert costs[gy, gx] >= 50


def test_turn_requires_a_fresh_costmap():
    config = load_config()
    now = [1.0]
    tracker = Tracker(config, lambda *_: None, clock=lambda: now[0], costmap_timeout_s=1.0)
    tracker.running = True
    tracker.use_lidar = False
    tracker.update_odom((0, 0, 0), "world")
    tracker.update_depth(np.ones((10, 10)))
    tracker.turn_goal = (1.0, "world")
    assert tracker.command() == (0.0, 0.0, "costmap_stale")
    tracker.update_costmap(object(), stamp_ns=100)
    assert tracker._motion_input_error() is None
    now[0] += 1.01
    tracker.update_costmap(object(), stamp_ns=100)
    assert tracker.command() == (0.0, 0.0, "costmap_stale")
    tracker.update_costmap(object(), stamp_ns=101)
    assert tracker._motion_input_error() is None
    tracker.update_odom((0, 0, 0), "other")
    assert tracker._motion_input_error() == "costmap_stale"


def test_costmap_swept_footprint_checks_forward_reverse_and_rotation():
    from types import SimpleNamespace

    from dimos.plugins.path_tracker.collision import collision_veto
    from dimos.plugins.path_tracker.messages import OccupancyGrid

    tracker = Tracker(load_config(), lambda *_: None)
    for point, velocity in (
        ((0.50, 0.10), (0.4, 0.0)),
        ((-0.50, 0.10), (-0.4, 0.0)),
        ((0.20, 0.20), (0.0, 0.5)),
    ):
        data = np.zeros((40, 40), dtype=np.int8)
        gx, gy = np.floor((np.array(point) + 1) / 0.05).astype(int)
        data[gy, gx] = 100
        origin = SimpleNamespace(position=SimpleNamespace(x=-1, y=-1))
        info = SimpleNamespace(width=40, height=40, resolution=0.05, origin=origin)
        grid = OccupancyGrid(SimpleNamespace(info=info, data=data.ravel()))
        assert collision_veto(
            tracker.guard, (0.0, 0.0, 0.0), grid, None, np.ones((10, 10)), *velocity
        ) == "map"
