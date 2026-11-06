"""Project-local paths for files produced by a DimOS run."""

from __future__ import annotations

import os
from pathlib import Path

from dimos.constants import DIMOS_PROJECT_ROOT

DIMOS_RUN_ROOT = DIMOS_PROJECT_ROOT / "run"


def get_run_dir() -> Path:
    configured = os.environ.get("DIMOS_RUN_DIR")
    return Path(configured) if configured else DIMOS_RUN_ROOT / "standalone"


def get_run_asset_dir() -> Path:
    configured = os.environ.get("DIMOS_RUN_ASSET_DIR")
    return Path(configured) if configured else get_run_dir() / "assets"


def get_run_log_dir() -> Path:
    configured = os.environ.get("DIMOS_RUN_LOG_DIR")
    return Path(configured) if configured else get_run_dir() / "logs"


def configure_run_paths(run_id: str) -> tuple[Path, Path, Path]:
    run_dir = DIMOS_RUN_ROOT / run_id
    log_dir = run_dir / "logs"
    asset_dir = run_dir / "assets"
    ros_log_dir = log_dir / "ros"
    log_dir.mkdir(parents=True, exist_ok=True)
    asset_dir.mkdir(parents=True, exist_ok=True)
    ros_log_dir.mkdir(parents=True, exist_ok=True)
    os.environ["DIMOS_RUN_ID"] = run_id
    os.environ["DIMOS_RUN_DIR"] = str(run_dir)
    os.environ["DIMOS_RUN_LOG_DIR"] = str(log_dir)
    os.environ["DIMOS_RUN_ASSET_DIR"] = str(asset_dir)
    os.environ["ROS_LOG_DIR"] = str(ros_log_dir)
    return run_dir, log_dir, asset_dir
