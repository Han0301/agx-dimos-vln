"""VLN application adapter for the domain-independent Web transport."""

import asyncio
import logging
from functools import partial
from pathlib import Path

from fastapi import HTTPException, Request
from fastapi.responses import HTMLResponse, JSONResponse, StreamingResponse
from starlette.datastructures import UploadFile
from starlette.formparsers import MultiPartException, MultiPartParser
from httpx import HTTPError

from dimos.plugins._agent_common import load_config
from dimos.plugins.web_input.transport import create_app as create_transport
from .agent import TaskAgent

logger = logging.getLogger("dimos.plugins.web_input.app")

SEARCH_STATUS_LABELS = {
    "checking_current_view": "正在检查当前画面",
    "exploring": "正在转向扫描周围",
    "detecting_with_memory": "正在检测当前画面并查询记忆",
    "checking_memory_result": "未获得可用目标框，正在筛选历史视角",
    "memory_fallback_frontier": "没有可用回访视角，正在转入前沿探索",
    "checking_memory": "正在查询历史视角",
    "planning_memory_goal": "正在规划前往历史视角的路径",
    "tracking_memory_goal": "正在前往历史视角",
    "replanning_memory_goal": "正在重新规划历史视角路径",
    "facing_memory_goal": "正在转向历史位置，转正后规划前往",
    "turning_memory_goal": "正在对齐历史观察朝向",
    "selecting_frontier": "正在选择可达前沿",
    "planning_frontier": "正在规划前沿路径",
    "replanning_frontier": "正在重新规划前沿路径",
    "tracking_frontier": "正在前往前沿航点",
    "waiting_exploration_view": "已结束移动，正在等待新观测",
    "checking_exploration_view": "已停稳，正在检测新视角",
    "frontier_cooldown": "前沿执行失败，正在换点",
    "exploration_lookaround": "无可达前沿，正在转向补看",
    "exploring_toward_target": "目标深度无法定位，正在规划短程靠近",
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
    "provider_failure": "视觉服务暂不可用",
    "verification_unavailable": "暂时无法复核目标",
    "invalid_detection": "目标检测结果无效",
}


SEARCH_REASON_LABELS = {
    "no_reachable_frontier": "没有可达前沿",
    "no_new_view": "未取得停稳后的新观测",
    "no_observation": "未取得有效观测",
    "no_valid_exploration_map": "缺少有效探索地图",
    "no_fresh_lidar": "未收到新鲜的 LiDAR 数据",
    "exploration_lidar_stale": "LiDAR 数据已过期",
    "exploration_frame_changed": "里程计坐标系已变化",
    "observation_stale": "相机或里程计观测已过期",
    "odom_stale": "里程计数据已过期",
    "depth_stale": "深度数据已过期",
    "costmap_stale": "控制代价地图已过期",
    "turn_interrupted": "转向已中断",
    "turn_already_active": "已有转向动作正在执行",
    "tracker_stopped": "跟踪控制器已停止",
    "no_accepted_path": "未取得通过校验的路径",
    "tracking_timeout": "跟踪超时",
    "task_timeout": "任务超时",
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
    def progress_fields(progress):
        return {
            "attempts": progress.get("attempts"),
            "max_attempts": progress.get("max_attempts"),
            **{key: progress[key] for key in (
                "source_stamp_ns", "memory_lookup_id", "leg_id", "leg_kind", "reason",
            ) if key in progress},
        }

    @staticmethod
    def result_fields(result):
        navigation = result.get("navigation") if isinstance(result, dict) else None
        if not isinstance(navigation, dict):
            return {"navigation_status": None}
        fields = {"navigation_status": navigation.get("status")}
        search = navigation.get("search")
        if isinstance(search, dict) and search.get("reason"):
            fields["search_reason"] = search["reason"]
        return fields

    @staticmethod
    def status(task):
        state = task["status"]
        progress = task.get("progress", {})
        stage = progress.get("status")
        reason = progress.get("reason")
        result = task["result"]
        navigation = result.get("navigation") if isinstance(result, dict) else None
        if state == "succeeded" and isinstance(navigation, dict):
            final_stage = navigation.get("status")
            if progress or final_stage in {
                "arrived",
                "within_standoff",
                "not_visible",
                "provider_failure",
                "invalid_detection",
                "invalid_depth",
                "target_mismatch",
                "exploration_unavailable",
                "search_exhausted",
                "verification_unavailable",
            }:
                if final_stage != stage:
                    reason = None
                stage = final_stage
                search = navigation.get("search")
                if isinstance(search, dict) and search.get("reason"):
                    reason = search["reason"]
        if stage in SEARCH_STATUS_LABELS:
            label = SEARCH_STATUS_LABELS[stage]
            leg_id = progress.get("leg_id")
            if stage == progress.get("status") and leg_id is not None:
                label = f"第 {leg_id} 段 · {label}"
            reason_label = SEARCH_REASON_LABELS.get(reason)
            if reason and reason.startswith("collision_"):
                reason_label = "碰撞检查阻止运动"
            if stage == "exploration_unavailable" and reason:
                label = "探索已停止"
                reason_label = reason_label or reason
            if stage == "frontier_cooldown" and reason:
                reason_label = reason_label or reason
            if reason_label and stage != "exploration_lookaround":
                label = f"{label}（{reason_label}）"
            previous = progress.get("last_frontier_failure")
            if stage in {"selecting_frontier", "planning_frontier", "replanning_frontier"} and previous:
                previous_reason = previous.get("reason")
                previous_leg = previous.get("leg_id")
                if previous_reason and previous_leg is not None:
                    detail = SEARCH_REASON_LABELS.get(previous_reason, previous_reason)
                    label = f"{label}（第 {previous_leg} 段{detail}）"
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
                **({"reason": reason} if reason else {}),
            }
        label = "navigating" if state == "running" else state
        return {"status": f"{label}: {task['goal']}"}


def _execute(runner, goal, image, *, cancel, on_progress):
    run = getattr(runner, "run_with_reference", None) if image is not None else None
    if run is None:
        run = getattr(runner, "run_with_progress", None)
    if run is None:
        return runner.run(goal, image, cancel=cancel)
    return run(goal, image, cancel=cancel, on_progress=on_progress)


def _cancel_motion(runner, cancel):
    callback = getattr(runner, "cancel_motion", None)
    if callback is not None:
        callback(cancel)


def _register_routes(policy, app, *, get_runner, enqueue, submit, bounded_stream,
                     task_snapshot, cancel_matching):
    browser_task_id = None

    @app.get("/", response_class=HTMLResponse)
    def index():
        return Path(__file__).with_name("index.html").read_text(encoding="utf-8")

    @app.post("/navigate", status_code=202)
    async def navigate(request: Request):
        nonlocal browser_task_id
        content_type = request.headers.get("content-type", "").partition(";")[0]
        if content_type.strip().lower() != "multipart/form-data":
            return await submit(request)

        try:
            form = await MultiPartParser(
                request.headers, bounded_stream(request), max_files=1, max_fields=1
            ).parse()
            try:
                upload = form.get("image")
                if upload is not None and not isinstance(upload, UploadFile):
                    raise HTTPException(422, "Expected an image file")
                image = await upload.read() if upload is not None else None
                task = enqueue(form.get("goal"), image, source="browser")
                browser_task_id = task["task_id"]
            finally:
                await form.close()
        except MultiPartException as exc:
            return JSONResponse({"success": False, "message": exc.message}, 422)
        except HTTPException as exc:
            return JSONResponse(
                {"success": False, "message": exc.detail}, exc.status_code
            )
        return JSONResponse(
            {"success": True, "goal": task["goal"], "task_id": task["task_id"]}
        )

    @app.post("/stop")
    async def stop():
        stop_all = getattr(get_runner(), "cancel_motion", None) is not None
        cancel_matching(lambda source: stop_all or source == "browser", source="web_stop")
        return {"success": True}

    @app.get("/status")
    async def browser_status(task_id: str | None = None):
        task = task_snapshot(task_id if task_id is not None else browser_task_id)
        if task is None or task["source"] != "browser":
            if task_id is not None:
                raise HTTPException(404, "Browser task unknown or evicted")
            return {"status": "idle"}
        if task["status"] == "cancelled":
            return {"status": "stopped"}
        if task["status"] == "failed":
            return {"status": f"failed: {task['error']}"}
        return policy.status(task)

    @app.get("/camera_feed")
    @app.get("/frontier_feed")
    async def unavailable_feed():
        raise HTTPException(404, "No source for this feed")

    @app.get("/realsense_feed")
    @app.get("/depth_feed")
    async def realsense_feed(request: Request):
        camera_frame = getattr(get_runner(), "camera_frame", None)
        if camera_frame is None:
            raise HTTPException(404, "The profile has no camera source")
        depth = request.url.path == "/depth_feed"

        async def stream():
            while True:
                frame = await asyncio.to_thread(camera_frame, depth)
                if frame is not None:
                    yield (
                        b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                    )
                await asyncio.sleep(0.1)

        return StreamingResponse(
            stream(), media_type="multipart/x-mixed-replace; boundary=frame"
        )

def create_app(config, runner_factory=None, *, llm_preload_timeout_s=None):
    if runner_factory is None:
        def runner_factory():
            return TaskAgent.from_config(load_config("vln_task", config["task_config"]))
    policy = VLNWebPolicy(llm_preload_timeout_s)
    return create_transport(
        config, runner_factory, startup=policy.startup, health_details=policy.health,
        result_fields=policy.result_fields, progress_fields=policy.progress_fields,
        execute=_execute, on_cancel=_cancel_motion,
        register_routes=partial(_register_routes, policy), title="Standalone VLN task input",
    )
