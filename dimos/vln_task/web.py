"""VLN application adapter for the domain-independent Web transport."""

import logging
from httpx import HTTPError

from dimos.plugins._agent_common import load_config
from dimos.plugins.web_input.transport import create_app as create_transport
from .agent import TaskAgent

logger = logging.getLogger("dimos.plugins.web_input.app")

SEARCH_STATUS_LABELS = {
    "checking_current_view": "正在检查当前画面",
    "exploring": "正在转向扫描周围",
    "exploring_toward_target": "正在朝目标方向换视角以恢复深度",
    "exploration_unavailable": "探索已停止，当前无法找到可达探索点",
    "invalid_depth": "目标深度无效",
    "target_mismatch": "候选与目标不符",
    "searching": "搜索中",
    "confirming": "正在复核候选目标",
    "confirmed": "已确认目标",
    "planning": "正在规划接近路径",
    "tracking": "正在跟踪路径",
    "replanning": "正在更新接近路径",
    "arrived": "已到达接近点",
    "localized": "已确认目标并定位",
    "preview_ready": "已确认目标，路径预览就绪",
    "within_standoff": "已确认目标，当前距离已足够接近",
    "search_exhausted": "搜索结束，未找到目标",
    "not_visible": "当前未找到目标",
    "verification_unavailable": "暂时无法复核目标",
    "invalid_detection": "目标检测结果无效",
}


class VLNWebPolicy:
    def __init__(self, timeout):
        self.timeout = timeout
        self.llm_warmup = {"status": "warming"} if timeout is not None else None

    def startup(self, runner, lock):
        if self.llm_warmup is None:
            return
        try:
            metrics = runner.preload_llm(self.timeout)
        except (HTTPError, ValueError) as exc:
            logger.warning(
                "Ollama preload failed error_type=%s", type(exc).__name__
            )
            with lock:
                self.llm_warmup["status"] = "failed"
        else:
            with lock:
                self.llm_warmup.update(metrics, status="ready")

    def health(self):
        return {} if self.llm_warmup is None else {"llm_warmup": self.llm_warmup.copy()}

    @staticmethod
    def result_fields(result):
        navigation = result.get("navigation") if isinstance(result, dict) else None
        return {"navigation_status": navigation.get("status") if isinstance(navigation, dict) else None}

    @staticmethod
    def status(task):
        state = task["status"]
        progress = task.get("progress", {})
        stage = progress.get("status")
        result = task["result"]
        navigation = result.get("navigation") if isinstance(result, dict) else None
        if state == "succeeded" and isinstance(navigation, dict):
            final_stage = navigation.get("status")
            if progress or final_stage in {
                "arrived",
                "within_standoff",
                "not_visible",
                "invalid_detection",
                "invalid_depth",
                "target_mismatch",
                "exploration_unavailable",
                "search_exhausted",
                "verification_unavailable",
            }:
                stage = final_stage
        if stage in SEARCH_STATUS_LABELS:
            label = SEARCH_STATUS_LABELS[stage]
            if stage in {"searching", "confirming"}:
                attempts, limit = (
                    progress.get("attempts"),
                    progress.get("max_attempts"),
                )
                if attempts is not None and limit is not None:
                    label = f"{label}（{attempts}/{limit}）"
            return {
                "status": f"{label} · {task['goal']}",
                "stage": stage,
                "task_id": task["task_id"],
            }
        label = "navigating" if state == "running" else state
        return {"status": f"{label}: {task['goal']}"}


def create_app(config, runner_factory=None, *, llm_preload_timeout_s=None):
    if runner_factory is None:
        def runner_factory():
            return TaskAgent.from_config(load_config("vln_task", config["task_config"]))
    policy = VLNWebPolicy(llm_preload_timeout_s)
    return create_transport(
        config, runner_factory, startup=policy.startup, health_details=policy.health,
        format_status=policy.status, result_fields=policy.result_fields,
    )
