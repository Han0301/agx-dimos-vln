"""Task-scoped uploads, bounded queue, observable completion and cancellation."""

import asyncio
import base64
import binascii
import json
import logging
import queue
import threading
import time
import traceback
import uuid
from contextlib import asynccontextmanager
from functools import partial

from fastapi import FastAPI, HTTPException, Query, Request

from dimos.plugins._agent_common import TaskCancelled, validate_image
from dimos.plugins.runtime_log import event, recording_enabled, task_context
from dimos.plugins.runtime_log._redaction import redact_text

logger = logging.getLogger("dimos.plugins.web_input.app")

def create_app(config, runner_factory, *, startup=None, health_details=None,
               result_fields=None, progress_fields=None, execute=None, on_cancel=None, register_routes=None,
               title="Task input"):
    pending = queue.Queue(maxsize=config["queue_size"])
    records, lock = {}, threading.RLock()
    stopping = threading.Event()
    worker_thread = None
    runner = None

    def report_progress(task, progress):
        with lock:
            if task["status"] == "running" and not task["cancel"].is_set():
                previous = task.get("progress", {})
                task["progress"] = dict(progress)
                if progress.get("status") != previous.get("status"):
                    event(logger, "task_stage", stage=progress.get("status"),
                          **(progress_fields(progress) if progress_fields is not None else {}))

    def worker(runner, loop):
        run_with_progress = getattr(runner, "run_with_progress", None)
        try:
            if startup is not None and not stopping.is_set():
                startup(runner, lock)
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
                            if execute is not None:
                                result = execute(
                                    runner, task["goal"], image, cancel=task["cancel"],
                                    on_progress=partial(report_progress, task),
                                )
                            elif run_with_progress is None:
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
                                    if recording_enabled():
                                        event(
                                            logger, "task_failed", error_type=type(exc).__name__,
                                            error_message=redact_text(str(exc))[:1000],
                                            stage=task.get("progress", {}).get("status"),
                                            traceback=[
                                                {"file": item.filename, "line": item.lineno, "function": item.name}
                                                for item in traceback.extract_tb(exc.__traceback__, limit=-12)
                                            ],
                                        )
                        finally:
                            with lock:
                                if task["finished_at"] is None:
                                    task["finished_at"] = time.time()
                                status = task["status"]
                                result = task["result"]
                            event(
                                logger, "task_finished", task_status=status,
                                **(result_fields(result) if result_fields is not None else {}),
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
        nonlocal worker_thread, runner
        runner = runner_factory()
        worker_thread = threading.Thread(
            target=worker,
            args=(runner, asyncio.get_running_loop()),
            name="web-task-worker",
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

    app = FastAPI(title=title, lifespan=lifespan)

    def public(task):
        return {
            key: value
            for key, value in task.items()
            if key not in {"image", "cancel", "done", "source", "queued_monotonic"}
        }

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
            if health_details is not None:
                result.update(health_details())
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

    def enqueue(goal, image, *, source="api"):
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
            "source": source,
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
            with task_context(task["task_id"]):
                logger.info(
                    "task queued has_image=%s source=%s", image is not None, source
                )
                event(logger, "task_submitted", goal_chars=len(goal), has_image=image is not None,
                      source=source)
            return public(task)

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
        if on_cancel is not None:
            for task in active:
                with task_context(task["task_id"]):
                    on_cancel(runner, task["cancel"])

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

    if register_routes is not None:
        def task_snapshot(task_id):
            with lock:
                task = records.get(task_id)
                return None if task is None else {**public(task), "source": task["source"]}

        def cancel_matching(predicate, *, source):
            with lock:
                selected = tuple(task for task in records.values() if predicate(task["source"]))
            cancel_tasks(selected, source=source)

        register_routes(
            app, get_runner=lambda: runner, enqueue=enqueue, submit=submit,
            bounded_stream=bounded_stream, task_snapshot=task_snapshot,
            cancel_matching=cancel_matching,
        )
    return app
