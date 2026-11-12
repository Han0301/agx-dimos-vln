from pathlib import Path

import numpy as np

from dimos.core.config import load_config as load_body
from dimos.plugins.path_tracker.config import load_config
from dimos.plugins.path_tracker.tracker import Tracker

ROOT = Path(__file__).resolve().parents[1]


def test_vln_motion_turns_without_lidar():
    config = load_config(ROOT / "config/plugins/path_tracker/tracker_motion.yaml")
    assert config["topics"]["lidar"] is None
    tracker = Tracker(config, lambda *_: None, clock=lambda: 10.0, depth_timeout_s=0.5)
    tracker.running = True
    tracker.update_odom((0, 0, 0), "world")
    tracker.update_depth(np.ones((10, 10)))
    tracker.turn_goal = (1.0, "world")
    assert tracker.command() == (0.0, 0.2, "exploration_turn")


def test_vln_body_profile_disables_builtin_avoidance():
    assert load_body(ROOT / "config/base_motion.yaml")["body"]["obstacle_avoidance"] is False
    assert "obstacle_avoidance" not in load_body(ROOT / "config/base.yaml")["body"]
