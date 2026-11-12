"""Strict plugin configuration; checks do not import ROS or SciPy."""

import math
from pathlib import Path

import yaml

from dimos.core.config import ROOT

DEFAULT = ROOT / "config/plugins/path_tracker/tracker.yaml"


def load_config(path=DEFAULT):
    cfg = yaml.safe_load(Path(path).read_text())
    expected = {
        "plugin",
        "output_only",
        "topics",
        "controller",
        "collision",
        "depth_scale",
    }
    if (
        not isinstance(cfg, dict)
        or not expected <= cfg.keys() <= expected | {"costmap_lidar"}
        or cfg["plugin"] != "path_tracker"
    ):
        raise ValueError(f"expected path_tracker config sections: {sorted(expected)}")
    if type(cfg["output_only"]) is not bool:
        raise ValueError("output_only must be boolean")
    topics = cfg["topics"]
    if not isinstance(topics, dict) or set(topics) != {
        "result",
        "odom",
        "depth",
        "lidar",
        "costmap",
        "cmd_vel",
    }:
        raise ValueError("invalid topic keys")
    for name, value in topics.items():
        if name in {"lidar", "costmap"} and value is None:
            continue
        if not isinstance(value, str) or not value.startswith("/") or value == "/":
            raise ValueError(f"invalid topic: {name}")
    lidar_source = cfg.get("costmap_lidar")
    if lidar_source is not None and (
        not isinstance(lidar_source, str)
        or not lidar_source.startswith("/")
        or lidar_source == "/"
        or topics["costmap"] is None
        or lidar_source == topics["costmap"]
        or topics["lidar"] is not None
    ):
        raise ValueError("costmap_lidar requires a separate costmap topic and lidar: null")
    expected_controller = {
        "tick_rate_hz",
        "mpc_horizon",
        "mpc_desired_v",
        "mpc_v_max",
        "mpc_w_max",
        "max_angular_speed",
        "decel_start_s",
        "stop_s",
    }
    expected_collision = {
        "cost_threshold",
        "robot_length",
        "robot_half_width",
        "robot_radius",
        "depth_obstacle_m",
        "horizon_m",
        "collision_penalty",
    }
    for section, keys in [
        ("controller", expected_controller),
        ("collision", expected_collision),
    ]:
        if not isinstance(cfg[section], dict) or set(cfg[section]) != keys:
            raise ValueError(f"invalid {section} keys")
        for name, value in cfg[section].items():
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value < 0
                or (value == 0 and name != "depth_obstacle_m")
            ):
                raise ValueError(f"{section}.{name} must be positive and finite")
    c = cfg["controller"]
    if type(c["mpc_horizon"]) is not int or c["decel_start_s"] >= c["stop_s"]:
        raise ValueError("invalid horizon/deceleration timing")
    if (
        isinstance(cfg["depth_scale"], bool)
        or not isinstance(cfg["depth_scale"], (int, float))
        or not math.isfinite(cfg["depth_scale"])
        or cfg["depth_scale"] <= 0
    ):
        raise ValueError("depth_scale must be positive and finite")
    return cfg
