"""Access the sibling DimOS Simple spatial memory in its own interpreter."""

import base64
import json
import logging
import math
import os
import selectors
import subprocess
import tempfile
import time
from contextlib import nullcontext
from dataclasses import dataclass
from pathlib import Path
from uuid import uuid4

from dimos.plugins._agent_common import TaskCancelled
from dimos.plugins.runtime_log import event, recording_enabled
from dimos.plugins.runtime_log._redaction import redact_text

from ._archive import MemoryArchive

logger = logging.getLogger(__name__)

_WORKER_PATH = Path(__file__).with_name("simple_memory_worker.py")
_DEFAULT_SIMPLE_ROOT = Path(__file__).resolve().parents[4] / "dimos_simple"
_START_TIMEOUT_S = 65.0
_OP_TIMEOUT_S = 3.0
_MAX_RESPONSE_BYTES = 64 * 1024


class SimpleMemoryUnavailable(RuntimeError):
    """The optional Simple memory worker could not service a request."""

    def __init__(self, message, *, reason="worker_error"):
        super().__init__(message)
        self.reason = reason


def _error_reason(exc):
    if isinstance(exc, SimpleMemoryUnavailable):
        return exc.reason
    if isinstance(exc, TaskCancelled):
        return "task_cancelled"
    if isinstance(exc, TimeoutError):
        return "task_timeout"
    return "worker_error"


@dataclass(frozen=True)
class MemoryHit:
    memory_id: str | None
    x: float
    y: float
    yaw: float
    similarity: float


def _query_hits(results):
    if not isinstance(results, list):
        raise SimpleMemoryUnavailable("Simple memory returned invalid search results", reason="invalid_results")
    hits = []
    for result in results:
        if not isinstance(result, dict):
            continue
        metadata = result.get("metadata")
        if isinstance(metadata, list) and metadata:
            metadata = metadata[0]
        if not isinstance(metadata, dict):
            continue
        values = (metadata.get("pos_x"), metadata.get("pos_y"), metadata.get("rot_z"), result.get("distance"))
        if not all(type(value) in (float, int) and math.isfinite(value) for value in values):
            continue
        x, y, yaw, distance = values
        memory_id = result.get("id") or metadata.get("frame_id")
        hits.append(MemoryHit(
            memory_id if isinstance(memory_id, str) else None,
            float(x), float(y), float(yaw), 1.0 - distance,
        ))
    return hits


def _archive_requested():
    return recording_enabled() or bool(os.environ.get("DIMOS_LOG_ROOT"))


class SimpleMemory:
    """Run one request at a time. Reset clears the current odometry session."""

    def __init__(self):
        self.root = Path(os.environ.get("DIMOS_SIMPLE_ROOT") or _DEFAULT_SIMPLE_ROOT).expanduser().resolve()
        self._runtime = None
        self._archive = None
        self._process = None
        self._pending = bytearray()

    def preload(self, remaining):
        return self._request({"action": "preload"}, remaining)

    def record(self, jpeg, pose, remaining, *, min_distance_m, min_yaw_delta_deg,
               min_interval_s, sample_group=None, stamp_ns=None, observation_id=None):
        """Store JPEG and XYZ/XYZW pose subject to the supplied sampling limits."""
        values = list(pose)
        payload = {
            "action": "record", "jpeg": base64.b64encode(jpeg).decode("ascii"),
            "pose": values, "min_distance_m": min_distance_m,
            "min_yaw_delta_deg": min_yaw_delta_deg, "min_interval_s": min_interval_s,
        }
        if sample_group is not None:
            payload.update(sample_group=sample_group, sample_stamp_ns=stamp_ns)
        if not _archive_requested():
            result = self._request(payload, remaining)
            return result.get("stored") is True if isinstance(result, dict) else result is True

        started = time.monotonic()
        payload["diagnostics"] = True
        payload["memory_request_id"] = uuid4().hex
        log_values = [value if math.isfinite(value) else None for value in values]
        fields = {
            "memory_request_id": payload["memory_request_id"],
            "observation_id": observation_id,
            "source_stamp_ns": stamp_ns,
            "image_bytes": len(jpeg),
            "position_xyz": log_values[:3],
            "orientation_xyzw": log_values[3:],
        }
        event(logger, "simple_memory_sample_started", **fields)
        try:
            result = self._request(payload, remaining)
        except Exception as exc:
            event(logger, "simple_memory_sample", **fields, outcome="error",
                  reason=_error_reason(exc), error_type=type(exc).__name__,
                  elapsed_ms=(time.monotonic() - started) * 1000)
            raise
        if not isinstance(result, dict):
            result = {"stored": result is True}
        stored = result.get("stored") is True
        memory_id = result.get("memory_id")
        image_path, image_outcome = None, "not_sampled"
        if stored:
            if self._archive is None:
                self._archive = MemoryArchive()
            image_path, image_outcome = self._archive.offer(memory_id, jpeg)
        event(
            logger, "simple_memory_sample", **fields, outcome="stored" if stored else "skipped",
            memory_id=memory_id, image_path=image_path, image_outcome=image_outcome,
            image_width=result.get("image_width"), image_height=result.get("image_height"),
            reason=result.get("reason"), distance_m=result.get("distance_m"),
            interval_s=result.get("interval_s"), min_distance_m=result.get("min_distance_m"),
            min_interval_s=result.get("min_interval_s"),
            yaw_delta_deg=result.get("yaw_delta_deg"), min_yaw_delta_deg=result.get("min_yaw_delta_deg"),
            elapsed_ms=(time.monotonic() - started) * 1000,
        )
        return stored

    def query(self, target, remaining):
        return [(hit.x, hit.y, hit.yaw, hit.similarity) for hit in self.query_hits(target, remaining)]

    def query_hits(self, target, remaining):
        payload = {"action": "query", "text": target}
        if _archive_requested():
            payload["memory_request_id"] = uuid4().hex
        if not recording_enabled():
            return _query_hits(self._request(payload, remaining))

        started = time.monotonic()
        query_text = redact_text(target)[:1000] if isinstance(target, str) else None
        fields = {
            "memory_request_id": payload["memory_request_id"], "target": query_text,
            "query_text": query_text, "backend": "clip_chroma",
        }
        event(logger, "simple_memory_lookup_started", **fields)
        try:
            results = self._request(payload, remaining)
            hits = _query_hits(results)
        except Exception as exc:
            event(logger, "simple_memory_lookup", **fields, outcome="error", reason=_error_reason(exc),
                  error_type=type(exc).__name__, elapsed_ms=(time.monotonic() - started) * 1000)
            raise
        event(
            logger, "simple_memory_lookup", **fields, outcome="hit" if hits else "miss",
            reason=None if hits else "no_valid_candidates" if results else "no_results",
            returned_candidates=len(results), candidate_ids=[hit.memory_id for hit in hits], candidates=len(hits),
            candidate_details=[
                {"rank": rank, "memory_id": hit.memory_id, "xy": [hit.x, hit.y],
                 "yaw": hit.yaw, "similarity": hit.similarity}
                for rank, hit in enumerate(hits, 1)
            ],
            elapsed_ms=(time.monotonic() - started) * 1000,
        )
        return hits

    def _start(self, remaining):
        python = self.root / ".venv/bin/python"
        if not python.is_file():
            raise SimpleMemoryUnavailable(
                f"Simple memory interpreter is missing at {python}", reason="interpreter_missing",
            )
        self._runtime = tempfile.TemporaryDirectory(prefix="dimos-simple-memory-")
        directory = self._runtime.name
        env = {**os.environ, "DIMOS_RUN_DIR": directory, "DIMOS_RUN_LOG_DIR": directory,
               "PYTHONPATH": str(self.root)}
        env.setdefault("HF_HUB_OFFLINE", "1")
        env.setdefault("TRANSFORMERS_OFFLINE", "1")
        recording = _archive_requested()
        if recording and self._archive is None:
            self._archive = MemoryArchive()
        stderr_target = self._worker_stderr() if recording else nullcontext(subprocess.DEVNULL)
        worker_args = [str(python), "-B", "-u", str(_WORKER_PATH), directory]
        if recording:
            worker_args.append("--log")
        started = time.monotonic() if recording else None
        with stderr_target as stderr:
            self._process = subprocess.Popen(
                worker_args,
                cwd=self.root, env=env, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                stderr=stderr, bufsize=0,
            )
            if recording:
                stderr_path = self._archive.relative_path("stderr.log") if stderr != subprocess.DEVNULL else None
                event(logger, "simple_memory_worker", phase="starting",
                      worker_pid=self._process.pid, stderr_path=stderr_path)
        os.set_blocking(self._process.stdin.fileno(), False)
        os.set_blocking(self._process.stdout.fileno(), False)
        ready = self._receive(remaining, time.monotonic() + _START_TIMEOUT_S)
        if recording:
            event(logger, "simple_memory_worker", phase="ready", worker_pid=self._process.pid,
                  stderr_path=stderr_path, backend=ready.get("backend") if isinstance(ready, dict) else None, elapsed_ms=(time.monotonic() - started) * 1000)

    def _worker_stderr(self):
        if self._archive.run is not None:
            try:
                return (self._archive.run / "stderr.log").open("ab")
            except OSError as exc:
                event(logger, "simple_memory_archive_unavailable", reason="worker_stderr_open_failed",
                      error_type=type(exc).__name__)
        return nullcontext(subprocess.DEVNULL)

    @staticmethod
    def _wait(stream, mask, remaining, deadline):
        with selectors.DefaultSelector() as selector:
            selector.register(stream, mask)
            while True:
                task_left = remaining()
                left = deadline - time.monotonic()
                if task_left <= 0:
                    raise TimeoutError("Task deadline exceeded")
                if left <= 0:
                    raise SimpleMemoryUnavailable("Simple memory worker timed out", reason="worker_timeout")
                if selector.select(min(task_left, left, 0.05)):
                    return

    def _receive(self, remaining, deadline):
        stdout = self._process.stdout
        while (index := self._pending.find(b"\n")) < 0:
            self._wait(stdout, selectors.EVENT_READ, remaining, deadline)
            chunk = os.read(stdout.fileno(), 4096)
            if not chunk:
                raise SimpleMemoryUnavailable("Simple memory worker closed unexpectedly", reason="worker_closed")
            self._pending.extend(chunk)
            if len(self._pending) > _MAX_RESPONSE_BYTES:
                raise SimpleMemoryUnavailable("Simple memory worker reply is too large", reason="reply_too_large")
        payload = self._pending[:index]
        del self._pending[:index + 1]
        response = json.loads(payload)
        if not isinstance(response, dict) or response.get("ok") is not True:
            detail = response.get("error", "invalid reply") if isinstance(response, dict) else "invalid reply"
            raise SimpleMemoryUnavailable(f"Simple memory worker failed: {detail}", reason="worker_reported_error")
        return response.get("result")

    def _request(self, payload, remaining):
        remaining()
        try:
            if self._process is None:
                self._start(remaining)
            deadline = time.monotonic() + _OP_TIMEOUT_S
            pending = memoryview(json.dumps(payload, allow_nan=False).encode("utf-8") + b"\n")
            stdin = self._process.stdin
            while pending:
                self._wait(stdin, selectors.EVENT_WRITE, remaining, deadline)
                pending = pending[os.write(stdin.fileno(), pending):]
            return self._receive(remaining, deadline)
        except TimeoutError:
            self.reset(reason="task_timeout")
            raise
        except (OSError, ValueError) as exc:
            self.reset(reason="ipc_failure")
            raise SimpleMemoryUnavailable(f"Simple memory IPC failed: {exc}", reason="ipc_failure") from exc
        except Exception as exc:
            self.reset(reason=_error_reason(exc))
            raise

    def reset(self, *, reason="reset"):
        if self._process is not None:
            process, self._process = self._process, None
            process.stdin.close()
            try:
                process.wait(timeout=0.25)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=0.25)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            process.stdout.close()
            if recording_enabled():
                event(logger, "simple_memory_worker", phase="stopped", worker_pid=process.pid,
                      return_code=process.returncode, reason=reason)
        self._pending.clear()
        if self._runtime is not None:
            runtime, self._runtime = self._runtime, None
            runtime.cleanup()

    def close(self):
        try:
            self.reset(reason="close")
        finally:
            if self._archive is not None:
                self._archive.close()
                self._archive = None
