"""Task-scoped uploads, bounded queue, observable completion and cancellation."""

import asyncio
import base64
import binascii
import json
import logging
import queue
import threading
import time
import uuid
from contextlib import asynccontextmanager
from functools import partial
from pathlib import Path

from fastapi import FastAPI, HTTPException, Query, Request
from fastapi.responses import HTMLResponse, JSONResponse, StreamingResponse
from httpx import HTTPError
from starlette.datastructures import UploadFile
from starlette.formparsers import MultiPartException, MultiPartParser

from dimos.plugins._agent_common import load_config, validate_image
from dimos.plugins.runtime_log import event, task_context
from dimos.plugins.vln_task.agent import TaskAgent, TaskCancelled

logger = logging.getLogger(__name__)

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


def create_app(config, runner_factory=None, *, llm_preload_timeout_s=None):
    pending = queue.Queue(maxsize=config["queue_size"])
    records, lock = {}, threading.RLock()
    stopping = threading.Event()
    worker_thread = None
    camera_frame = None
    cancel_motion = None
    browser_task_id = None
    llm_warmup = {"status": "warming"} if llm_preload_timeout_s is not None else None
    if runner_factory is None:

        def runner_factory():
            return TaskAgent.from_config(load_config("vln_task", config["task_config"]))

    def report_progress(task, progress):
        with lock:
            if task["status"] == "running" and not task["cancel"].is_set():
                previous = task.get("progress", {})
                task["progress"] = dict(progress)
                if progress.get("status") != previous.get("status"):
                    event(logger, "task_stage", stage=progress.get("status"),
                          attempts=progress.get("attempts"), max_attempts=progress.get("max_attempts"))

    def worker(runner, loop):
        run_with_progress = getattr(runner, "run_with_progress", None)
        try:
            if llm_warmup is not None and not stopping.is_set():
                try:
                    metrics = runner.preload_llm(llm_preload_timeout_s)
                except (HTTPError, ValueError) as exc:
                    logger.warning(
                        "Ollama preload failed error_type=%s", type(exc).__name__
                    )
                    with lock:
                        llm_warmup["status"] = "failed"
                else:
                    with lock:
                        llm_warmup.update(metrics, status="ready")
            while not stopping.is_set():
                try:
                    task = pending.get(timeout=0.1)
                except queue.Empty:
                    continue
                try:
                    with lock:
                        if task["cancel"].is_set():
                            continue
                        task["status"] = "running"
                        task["started_at"] = time.time()
                        image = task["image"]
                    with task_context(task["task_id"]):
                        started = time.perf_counter()
                        event(logger, "task_started", queue_ms=(time.monotonic() - task["queued_monotonic"]) * 1000)
                        logger.info(
                            "task started queue_ms=%.1f",
                            max(0, task["started_at"] - task["created_at"]) * 1000,
                        )
                        try:
                            if run_with_progress is None:
                                result = runner.run(
                                    task["goal"], image, cancel=task["cancel"]
                                )
                            else:
                                result = run_with_progress(
                                    task["goal"],
                                    image,
                                    cancel=task["cancel"],
                                    on_progress=partial(report_progress, task),
                                )
                            with lock:
                                if not task["cancel"].is_set():
                                    task["result"], task["status"] = result, "succeeded"
                        except TaskCancelled:
                            with lock:
                                task["status"] = "cancelled"
                        except Exception as exc:
                            with lock:
                                if not task["cancel"].is_set():
                                    task["status"], task["error"] = (
                                        "failed",
                                        f"{type(exc).__name__}: {exc}",
                                    )
                                    logger.warning(
                                        "task failed error_type=%s", type(exc).__name__
                                    )
                                    event(logger, "task_failed", error_type=type(exc).__name__)
                        finally:
                            with lock:
                                if task["finished_at"] is None:
                                    task["finished_at"] = time.time()
                                status = task["status"]
                                result = task["result"]
                            navigation = result.get("navigation") if isinstance(result, dict) else None
                            event(
                                logger, "task_finished", task_status=status,
                                navigation_status=navigation.get("status") if isinstance(navigation, dict) else None,
                                elapsed_ms=(time.perf_counter() - started) * 1000,
                            )
                            logger.info(
                                "task finished status=%s elapsed_ms=%.1f",
                                status,
                                (time.perf_counter() - started) * 1000,
                            )
                            loop.call_soon_threadsafe(task["done"].set)
                finally:
                    with lock:
                        task["image"] = None
                    pending.task_done()
        finally:
            if hasattr(runner, "__exit__"):
                runner.__exit__()

    @asynccontextmanager
    async def lifespan(app):
        nonlocal worker_thread, camera_frame, cancel_motion
        runner = runner_factory()
        camera_frame = getattr(runner, "camera_frame", None)
        cancel_motion = getattr(runner, "cancel_motion", None)
        worker_thread = threading.Thread(
            target=worker,
            args=(runner, asyncio.get_running_loop()),
            name="vln-task-worker",
        )
        worker_thread.start()
        logger.info("worker started queue_capacity=%d", config["queue_size"])
        try:
            yield
        finally:
            stopping.set()
            try:
                cancel_tasks(tuple(records.values()), source="web_shutdown")
            finally:
                # A provider call in flight completes or hits its configured timeout.
                await asyncio.to_thread(worker_thread.join)
                logger.info("worker stopped")

    app = FastAPI(title="Standalone VLN task input", lifespan=lifespan)

    def public(task):
        return {
            key: value
            for key, value in task.items()
            if key not in {"image", "cancel", "done", "browser", "queued_monotonic"}
        }

    @app.get("/", response_class=HTMLResponse)
    def index():
        return Path(__file__).with_name("index.html").read_text(encoding="utf-8")

    @app.get("/health")
    def health():
        with lock:
            running = sum(task["status"] == "running" for task in records.values())
            result = {
                "status": "ok",
                "queued": pending.qsize(),
                "running": running,
                "mode": "standalone",
                "worker_alive": bool(worker_thread and worker_thread.is_alive()),
            }
            if llm_warmup is not None:
                result["llm_warmup"] = llm_warmup.copy()
        return result

    async def bounded_stream(request):
        length = 0
        async for chunk in request.stream():
            length += len(chunk)
            if length > config["max_body_bytes"]:
                raise HTTPException(413, "Request body too large")
            yield chunk

    @app.post("/tasks", status_code=202)
    async def submit(request: Request):
        body = b"".join([chunk async for chunk in bounded_stream(request)])
        try:
            payload = json.loads(body)
            if not isinstance(payload, dict) or set(payload) - {"goal", "image_base64"}:
                raise ValueError("Expected goal and optional image_base64")
            goal = payload.get("goal")
            image = None
            if payload.get("image_base64") is not None:
                image = base64.b64decode(payload["image_base64"], validate=True)
        except (ValueError, TypeError, binascii.Error) as exc:
            raise HTTPException(422, str(exc)) from exc
        return enqueue(goal, image)

    def enqueue(goal, image, *, browser=False):
        nonlocal browser_task_id
        try:
            if not isinstance(goal, str) or not goal.strip() or len(goal) > 4000:
                raise ValueError("Goal must contain 1..4000 characters")
            if image is not None:
                validate_image(image)
        except ValueError as exc:
            raise HTTPException(422, str(exc)) from exc
        task = {
            "task_id": uuid.uuid4().hex,
            "goal": goal,
            "status": "queued",
            "result": None,
            "error": None,
            "created_at": time.time(),
            "queued_monotonic": time.monotonic(),
            "started_at": None,
            "finished_at": None,
            "image": image,
            "cancel": threading.Event(),
            "done": asyncio.Event(),
            "browser": browser,
        }
        with lock:
            if len(records) >= config["max_records"]:
                for key, item in list(records.items()):
                    if item["status"] in {"succeeded", "failed", "cancelled"}:
                        del records[key]
                        break
                else:
                    raise HTTPException(429, "Task record capacity reached")
            try:
                pending.put_nowait(task)
            except queue.Full as exc:
                raise HTTPException(429, "Task queue full") from exc
            records[task["task_id"]] = task
            if browser:
                browser_task_id = task["task_id"]
            with task_context(task["task_id"]):
                logger.info(
                    "task queued has_image=%s browser=%s", image is not None, browser
                )
                event(logger, "task_submitted", goal_chars=len(goal), has_image=image is not None,
                      source="browser" if browser else "api")
            return public(task)

    @app.post("/navigate", status_code=202)
    async def navigate(request: Request):
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
                task = enqueue(form.get("goal"), image, browser=True)
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

    def cancel_tasks(tasks, *, source):
        active = []
        with lock:
            for task in tasks:
                if task["status"] not in {"queued", "running"}:
                    continue
                was_running = task["status"] == "running"
                if was_running:
                    active.append(task)
                with task_context(task["task_id"]):
                    event(logger, "stop_requested", source=source, was_running=was_running)
                    task["cancel"].set()
                    task["status"] = "cancelled"
                    task["finished_at"] = time.time()
                    task["image"] = None
                    task["done"].set()
                    logger.info("task cancelled")
        if cancel_motion is not None:
            for task in active:
                with task_context(task["task_id"]):
                    cancel_motion(task["cancel"])

    @app.post("/stop")
    async def stop():
        # In motion mode, queued API goals must not restart motion after Stop.
        cancel_tasks(
            tuple(
                task for task in records.values()
                if task["browser"] or cancel_motion is not None
            ),
            source="web_stop",
        )
        return {"success": True}

    @app.get("/status")
    async def browser_status(task_id: str | None = None):
        with lock:
            task = records.get(task_id if task_id is not None else browser_task_id)
            if task is None or not task["browser"]:
                if task_id is not None:
                    raise HTTPException(404, "Browser task unknown or evicted")
                return {"status": "idle"}
            state = task["status"]
            if state == "cancelled":
                return {"status": "stopped"}
            if state == "failed":
                return {"status": f"failed: {task['error']}"}
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

    @app.get("/camera_feed")
    @app.get("/frontier_feed")
    async def unavailable_feed():
        raise HTTPException(404, "No source for this feed")

    @app.get("/realsense_feed")
    @app.get("/depth_feed")
    async def realsense_feed(request: Request):
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

    @app.get("/tasks/{task_id}")
    async def status(task_id: str, wait_s: float = Query(0, ge=0, le=20)):
        with lock:
            task = records.get(task_id)
            if task is None:
                raise HTTPException(404, "Task unknown or evicted")
        if wait_s:
            try:
                await asyncio.wait_for(task["done"].wait(), timeout=wait_s)
            except TimeoutError:
                pass
        with lock:
            return public(task)

    @app.post("/tasks/{task_id}/cancel")
    async def cancel(task_id: str):
        with lock:
            if task_id not in records:
                raise HTTPException(404, "Task unknown or evicted")
            task = records[task_id]
        cancel_tasks((task,), source="api_cancel")
        with lock:
            return public(task)

    return app
