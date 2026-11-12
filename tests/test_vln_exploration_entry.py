import json
import subprocess
from pathlib import Path

import pytest

from dimos.vln_task.web import VLNWebPolicy

ROOT = Path(__file__).resolve().parents[1]


def check_motion(*args):
    return subprocess.run(
        ["bash", str(ROOT / "sh/start_vln_motion.sh"), "--check", *args],
        cwd=ROOT.parent,
        capture_output=True,
        text=True,
        timeout=5,
    )


def test_memory_motion_flag_is_reported_by_configuration_check():
    result = check_motion("--exploration-memory")
    assert result.returncode == 0, result.stderr
    decoder = json.JSONDecoder()
    profile, index = decoder.raw_decode(result.stdout)
    tracker = decoder.decode(result.stdout[index:].lstrip())
    assert profile["exploration_memory"] is True
    assert profile["active_search"] is not None
    assert tracker["output_only"] is False


def test_default_motion_check_enables_memory():
    result = check_motion()
    assert result.returncode == 0, result.stderr
    profile, _ = json.JSONDecoder().raw_decode(result.stdout)
    assert profile["exploration_memory"] is True


@pytest.mark.parametrize(
    ("stage", "label"),
    [
        ("selecting_frontier", "正在选择可达前沿"),
        ("planning_frontier", "正在规划前沿路径"),
        ("replanning_frontier", "正在重新规划前沿路径"),
        ("tracking_frontier", "正在前往前沿航点"),
        ("waiting_exploration_view", "已结束移动，正在等待新观测"),
        ("checking_exploration_view", "已停稳，正在检测新视角"),
        ("frontier_cooldown", "前沿暂时冷却，正在换点"),
        ("exploration_lookaround", "无可达前沿，正在转向补看"),
    ],
)
def test_web_reports_exploration_phase_and_leg(stage, label):
    task = {
        "task_id": "task1",
        "status": "running",
        "goal": "找白色垃圾桶",
        "result": None,
        "progress": {"status": stage, "leg_id": 2},
    }

    assert VLNWebPolicy.status(task) == {
        "task_id": "task1",
        "stage": stage,
        "status": f"第 2 段 · {label} · 找白色垃圾桶",
    }


@pytest.mark.parametrize(
    ("stage", "reason", "detail"),
    [
        ("exploration_unavailable", "no_valid_exploration_map", "缺少有效探索地图"),
        ("exploration_unavailable", "no_fresh_lidar", "未收到新鲜的 LiDAR 数据"),
        ("search_exhausted", "no_reachable_frontier", "没有可达前沿"),
        ("search_exhausted", "no_new_view", "未取得停稳后的新观测"),
    ],
)
def test_web_reports_finished_search_reason(stage, reason, detail):
    result = {"navigation": {"status": stage, "search": {"reason": reason}}}
    task = {
        "task_id": "task1",
        "status": "succeeded",
        "goal": "找白色垃圾桶",
        "result": result,
        "progress": {"status": "planning_frontier", "leg_id": 2},
    }

    state = VLNWebPolicy.status(task)
    assert state["stage"] == stage
    assert state["reason"] == reason
    assert detail in state["status"]
    assert VLNWebPolicy.result_fields(result) == {
        "navigation_status": stage,
        "search_reason": reason,
    }


def test_web_preserves_the_previous_cooldown_when_selecting_the_next_frontier():
    task = {
        "task_id": "task1",
        "status": "running",
        "goal": "找白色垃圾桶",
        "result": None,
        "progress": {
            "status": "planning_frontier",
            "leg_id": 3,
            "last_frontier_failure": {"leg_id": 2, "reason": "tracking_timeout"},
        },
    }

    state = VLNWebPolicy.status(task)
    assert state["stage"] == "planning_frontier"
    assert "第 3 段" in state["status"]
    assert "第 2 段跟踪超时" in state["status"]


def test_web_translates_the_current_cooldown_reason():
    task = {
        "task_id": "task1",
        "status": "running",
        "goal": "找白色垃圾桶",
        "result": None,
        "progress": {"status": "frontier_cooldown", "leg_id": 2, "reason": "no_accepted_path"},
    }

    state = VLNWebPolicy.status(task)
    assert state["reason"] == "no_accepted_path"
    assert "未取得通过校验的路径" in state["status"]
