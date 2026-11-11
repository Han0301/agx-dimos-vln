"""Strict configuration for body communication and its native ROS endpoints."""

import math
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
MODULES = {"body"}
TOPICS = {
    "odom",
    "lidar",
    "cmd_vel",
    "tf",
    "lowstate",
}


def load_config(path):
    path = Path(path).resolve()
    with path.open() as f:
        cfg = yaml.safe_load(f)
    if not isinstance(cfg, dict):
        raise ValueError("configuration must be a mapping")
    required = {"version", "ros", "modules", "body", "camera", "topics"}
    if set(cfg) != required or cfg["version"] != 2:
        raise ValueError(
            f"expected version 2 body-only config (use current base.yaml); top-level keys: {sorted(required)}"
        )
    if set(cfg["modules"]) != MODULES:
        raise ValueError(f"modules must be exactly {sorted(MODULES)}")
    if any(type(v) is not bool for v in cfg["modules"].values()):
        raise ValueError("module switches must be booleans")
    if set(cfg["topics"]) != TOPICS:
        raise ValueError(f"topics must be exactly {sorted(TOPICS)}")
    if any(
        not isinstance(v, str) or not v.startswith("/") for v in cfg["topics"].values()
    ):
        raise ValueError("ROS topics must be absolute names")
    if len(set(cfg["topics"].values())) != len(TOPICS):
        raise ValueError("configured topics must not collide")
    expected_sections = {
        "ros": {"node_name", "use_sim_time"},
        "body": {
            "ip",
            "interface",
            "mode",
            "aes_key_env",
            "allow_motion",
            "command_timeout_s",
            "max_linear_mps",
            "max_angular_rps",
            "world_frame",
            "base_frame",
        },
        "camera": {"translation", "pitch", "mount_frame"},
    }
    for name, keys in expected_sections.items():
        if not isinstance(cfg[name], dict) or set(cfg[name]) != keys:
            raise ValueError(f"{name} keys must be exactly {sorted(keys)}")
    if type(cfg["ros"]["use_sim_time"]) is not bool:
        raise ValueError("ros.use_sim_time must be boolean")
    if type(cfg["body"]["allow_motion"]) is not bool:
        raise ValueError("body.allow_motion must be boolean")
    for name in ("command_timeout_s", "max_linear_mps", "max_angular_rps"):
        value = cfg["body"][name]
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value <= 0
        ):
            raise ValueError(f"body.{name} must be positive and finite")
    camera = cfg["camera"]
    if len(camera["translation"]) != 3 or not all(
        math.isfinite(v) for v in camera["translation"]
    ):
        raise ValueError("camera.translation must have three finite values")
    if not isinstance(camera["mount_frame"], str) or not camera["mount_frame"]:
        raise ValueError("camera.mount_frame is required")
    if not math.isfinite(camera["pitch"]):
        raise ValueError("camera.pitch must be finite")
    return cfg
