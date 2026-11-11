"""Explicit references to existing plugin configurations."""

import math
from pathlib import Path

import yaml

from dimos.config import load_config as load_body
from dimos.plugins._agent_common import ROOT, load_config
from dimos.plugins.path_tracker.config import load_config as load_tracker
from dimos.plugins.xnavdp.__main__ import load_test_config


def load_profile(path):
    cfg = yaml.safe_load(Path(path).read_text())
    refs = {
        "task_config",
        "web_config",
        "xnavdp_config",
        "tracker_config",
        "body_config",
    }
    if isinstance(cfg, dict) and "active_search_config" in cfg:
        refs.add("active_search_config")
    profile_keys = refs | {"version", "observation"}
    if isinstance(cfg, dict) and "llm_preload_timeout_s" in cfg:
        profile_keys.add("llm_preload_timeout_s")
    if (
        not isinstance(cfg, dict)
        or set(cfg) != profile_keys
        or type(cfg["version"]) is not int
        or cfg["version"] != 1
    ):
        raise ValueError(
            "Expected VLN profile version 1 with plugin references and observation settings"
        )
    llm_preload_timeout = cfg.get("llm_preload_timeout_s")
    if llm_preload_timeout is not None and (
        type(llm_preload_timeout) not in (int, float)
        or not math.isfinite(llm_preload_timeout)
        or llm_preload_timeout <= 0
    ):
        raise ValueError("llm_preload_timeout_s must be positive and finite or null")
    observation = cfg["observation"]
    if not isinstance(observation, dict) or set(observation) != {
        "gap_warning_ms",
        "cache_s",
        "approach_stop_m",
    }:
        raise ValueError("Invalid VLN observation settings")
    for key, value in observation.items():
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{key} must be positive and finite")
    paths = {}
    for key in refs:
        if not isinstance(cfg[key], str) or not cfg[key].strip():
            raise ValueError(f"{key} must reference a config file")
        paths[key] = ROOT / cfg[key]
    task = load_config("vln_task", paths["task_config"])
    tracker = load_tracker(paths["tracker_config"])
    if not tracker["output_only"]:
        raise ValueError("VLN preview requires tracker output_only")
    body = load_body(paths["body_config"])
    search = None
    if "active_search_config" in paths:
        from dimos.plugins.active_search.config import load_config as load_search

        search = load_search(paths["active_search_config"])
    return {
        "version": cfg["version"],
        "task": task,
        "llm": load_config("llm", task["llm_config"]),
        "llm_preload_timeout_s": llm_preload_timeout,
        "vlm": load_config("vlm", task["vlm_config"]),
        "web": load_config("web_input", paths["web_config"]),
        "xnavdp": load_test_config(paths["xnavdp_config"])["test"],
        "tracker": tracker,
        "camera": body["camera"],
        "observation": observation,
        "active_search": search,
    }
