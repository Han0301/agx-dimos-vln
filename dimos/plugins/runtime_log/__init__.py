"""Per-launch console, ROS and Python component logs."""

import copy
import json
import logging
import os
import re
import socket
import subprocess
import sys
import tempfile
import time
from contextlib import ExitStack, contextmanager, redirect_stderr, redirect_stdout
from contextvars import ContextVar
from datetime import datetime
from itertools import count
from pathlib import Path
from threading import Lock
from zoneinfo import ZoneInfo

from ._writer import BackgroundWriter, ConsoleFile, LogFile

PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_ROOT = PROJECT_ROOT / "data/log"
DEFAULT_EVIDENCE = "failures"
SPOOL_ROOT = PROJECT_ROOT / "run/logs/runtime_log"
LOCAL_TIME = ZoneInfo("Asia/Shanghai")
MODULE_NAME = re.compile(r"[a-z][a-z0-9_]*")
SCHEMA_VERSION = 2
EVENT_QUEUE_SIZE = 1024
_EVIDENCE_LIMIT = 16 * 1024 * 1024
_EVIDENCE_ITEM_LIMIT = 1024 * 1024
_DIAGNOSTIC_LIMIT = 128 * 1024 * 1024
_DIAGNOSTIC_ITEM_LIMIT = 4 * 1024 * 1024
_DIAGNOSTIC_FILE_LIMIT = 256
_EVIDENCE_NAME = re.compile(r"[a-zA-Z0-9][a-zA-Z0-9_.-]{0,95}")
_task_id = ContextVar("log_task_id", default=None)
_event_fields = ContextVar("log_event_fields", default={})
_active_components = None


def log_root(base=None):
    return (
        Path(base or os.environ.get("DIMOS_LOG_ROOT") or DEFAULT_ROOT)
        .expanduser()
        .resolve()
    )


def module_directory(module, base=None):
    if not MODULE_NAME.fullmatch(module):
        raise ValueError(f"Invalid runtime log module: {module!r}")
    return log_root(base) / module


def get_start_ticks(pid):
    try:
        fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
        return None if fields[0] == "Z" else fields[19]
    except (OSError, IndexError):
        return None


def _recording_session():
    try:
        session = json.loads((log_root() / ".active-session").read_text())
        ticks = get_start_ticks(int(session["pid"]))
        source = Path(session["source"]).resolve()
        if (
            ticks is not None
            and ticks == str(session.get("start_ticks", ticks))
            and source.parent == SPOOL_ROOT.resolve()
            and source.is_dir()
        ):
            return {**session, "source": source}
    except (OSError, ValueError, TypeError, KeyError, RuntimeError, OverflowError):
        pass
    return None


def environment_metadata():
    fields = {"host": socket.gethostname()}
    for name in ("ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "ROS_DISTRO"):
        value = os.environ.get(name)
        if value:
            fields[name.lower()] = value[:128]
    return fields


def create_run(module, *, base=None, ros=False, pid=None, metadata=None):
    if base is None:
        session = _recording_session()
        base = session["source"] if session else None
        if base is None and not os.environ.get("DIMOS_LOG_ROOT"):
            return None
    if callable(metadata):
        metadata = metadata()
    parent = module_directory(module, base)
    parent.mkdir(parents=True, exist_ok=True)
    pid = os.getpid() if pid is None else pid
    started = datetime.now(LOCAL_TIME)
    stamp = started.strftime("%Y%m%d_%H%M%S_%f")
    run = Path(tempfile.mkdtemp(prefix=f"{stamp}_{pid}_", dir=parent))
    if ros:
        (run / "ros").mkdir()
    (run / "run.json").write_text(
        json.dumps(
            {
                "schema_version": SCHEMA_VERSION,
                "module": module,
                "pid": pid,
                "metadata": {**environment_metadata(), **(metadata or {})},
                "start_ticks": get_start_ticks(pid),
                "started_ns": time.monotonic_ns(),
                "started_at": started.isoformat(timespec="milliseconds"),
            }
        )
        + "\n",
        encoding="utf-8",
    )
    return run


@contextmanager
def task_context(task_id):
    token = _task_id.set(task_id)
    try:
        yield
    finally:
        _task_id.reset(token)


def current_task_id():
    return _task_id.get()


@contextmanager
def event_context(**fields):
    token = _event_fields.set({**_event_fields.get(), **fields})
    try:
        yield
    finally:
        _event_fields.reset(token)


def recording_enabled():
    return _active_components is not None


def _snapshot_fields(fields):
    return {
        "task_id": _task_id.get(),
        **_event_fields.get(),
        **fields,
        "monotonic_ns": time.monotonic_ns(),
    }


def event(logger, name, **fields):
    """Offer a structured diagnostic without calling other logging handlers."""
    handler = _active_components
    if handler is not None:
        handler.event(logger.name, name, _snapshot_fields(fields))


def artifact(filename, data, **metadata):
    handler = _active_components
    if handler is not None and handler.evidence:
        return handler.artifact(filename, data, metadata)
    return None


def diagnostic_artifact(filename, data, **metadata):
    """Queue bounded replay data in any active recording."""
    handler = _active_components
    return handler.diagnostic_artifact(filename, data, metadata) if handler is not None else None


class _Mirror:
    def __init__(self, stream, writer):
        self._stream = stream
        self._writer = writer

    def __getattr__(self, name):
        return getattr(self._stream, name)

    def write(self, message):
        count = self._stream.write(message)
        if message:
            self._writer.offer(message.encode("utf-8", errors="backslashreplace"))
        return count

    def flush(self):
        self._stream.flush()


def _restore_stream(stream, fd, original):
    try:
        stream.flush()
    except OSError:
        pass
    finally:
        os.dup2(original, fd)


def _finish_mirror(process):
    try:
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()


def _mirror_fd(resources, stream, fd, filename):
    if stream.fileno() != fd:
        raise ValueError("Stream is not the standard file descriptor")
    stream.flush()
    original = os.dup(fd)
    resources.callback(os.close, original)
    environment = {
        **os.environ,
        "PYTHONDONTWRITEBYTECODE": "1",
        "PYTHONPATH": os.pathsep.join(filter(None, (str(PROJECT_ROOT), os.environ.get("PYTHONPATH")))),
    }
    process = subprocess.Popen(
        [sys.executable, "-m", "dimos.plugins.runtime_log.relay", str(filename)],
        env=environment,
        stdin=subprocess.PIPE,
        stdout=original,
        start_new_session=True,
    )
    resources.callback(_finish_mirror, process)
    with process.stdin:
        os.dup2(process.stdin.fileno(), fd)
    resources.callback(_restore_stream, stream, fd, original)


class _TaskConsoleFormatter(logging.Formatter):
    def format(self, record):
        component = record.name.removeprefix("dimos.").removeprefix("plugins.")
        task_id = _task_id.get()
        task = f" task_id={task_id}" if task_id is not None else ""
        return f"{record.levelname} [{component}]{task} {super().format(record)}"


_TASK_LOG_NAMESPACES = ("dimos.plugins", "dimos.path_tracker", "dimos.vln_task")


def _keep_http_access(record):
    # Uvicorn passes client, method, path (with query), HTTP version and status.
    if not isinstance(record.args, tuple) or len(record.args) != 5:
        return True
    _, method, path, _, status = record.args
    return not (
        method == "GET"
        and isinstance(path, str)
        and path.partition("?")[0] == "/status"
        and status == 200
    )


@contextmanager
def task_console():
    """Show Web VLN events even when file recording is disabled."""
    root = logging.getLogger()
    access = logging.getLogger("uvicorn.access")
    handler = logging.StreamHandler()
    handler.setFormatter(_TaskConsoleFormatter())
    prefixes = tuple(f"{name}." for name in _TASK_LOG_NAMESPACES)
    handler.addFilter(
        lambda record: record.levelno >= logging.WARNING
        or (
            record.levelno >= logging.INFO
            and (record.name in _TASK_LOG_NAMESPACES or record.name.startswith(prefixes))
        )
    )
    with ExitStack() as resources:
        resources.callback(handler.close)
        root.addHandler(handler)
        resources.callback(root.removeHandler, handler)
        access.addFilter(_keep_http_access)
        resources.callback(access.removeFilter, _keep_http_access)
        for name in _TASK_LOG_NAMESPACES:
            logger = logging.getLogger(name)
            resources.callback(logger.setLevel, logger.level)
            if logger.getEffectiveLevel() > logging.INFO:
                logger.setLevel(logging.INFO)
        yield


class _JsonFormatter(logging.Formatter):
    def format(self, record):
        fields = {
            **record.runtime_fields,
            "time": datetime.fromtimestamp(record.created, LOCAL_TIME).isoformat(
                timespec="milliseconds"
            ),
            "level": record.levelname,
            "logger": record.name,
            "message": record.getMessage(),
        }
        if fields.get("task_id") is None:
            fields.pop("task_id", None)
        if record.exc_info:
            fields["exception"] = self.formatException(record.exc_info)
        return json.dumps(fields, ensure_ascii=False, default=str, allow_nan=False)


class _Components(logging.Handler):
    def __init__(self, run, *, evidence):
        super().__init__()
        self._run_dir = run
        self._directory = run / "modules"
        self._files = {}
        self._formatter = _JsonFormatter()
        self.evidence = evidence == "failures"
        self._artifact_budget = _EVIDENCE_LIMIT
        self._artifact_count = count(1)
        self._artifact_lock = Lock()
        self._diagnostic_files = 0
        self._diagnostic_bytes = 0
        self._diagnostic_rejected = {}
        self._next_diagnostic_health = 0.0
        self._writer = BackgroundWriter(
            self._persist, self._finish, capacity=EVENT_QUEUE_SIZE, name="runtime-log",
            report=self._report,
        )

    def emit(self, record):
        snapshot = copy.copy(record)
        snapshot.runtime_fields = _snapshot_fields({})
        self._writer.offer(snapshot)
        fallback = logging.lastResort
        if not logging.root.handlers and fallback and record.levelno >= fallback.level:
            fallback.handle(record)

    def event(self, logger, name, fields):
        record = logging.LogRecord(logger, logging.INFO, "", 0, name, (), None)
        record.runtime_fields = {**fields, "event": name}
        return self._writer.offer(record)

    def artifact(self, filename, data, metadata):
        if not isinstance(filename, str) or not _EVIDENCE_NAME.fullmatch(filename):
            return None
        if not isinstance(data, (bytes, str)) or len(data) > _EVIDENCE_ITEM_LIMIT:
            return None
        if isinstance(data, str):
            data = data.encode("utf-8", errors="replace")
        with self._artifact_lock:
            if not 0 < len(data) <= min(_EVIDENCE_ITEM_LIMIT, self._artifact_budget):
                return None
            sequence = next(self._artifact_count)
            if sequence > 32:
                return None
            path = f"evidence/{sequence:03d}-{filename}"
            if not self._queue_artifact(path, data, metadata, "artifact_saved"):
                return None
            self._artifact_budget -= len(data)
            return path

    def _queue_artifact(self, path, data, metadata, name):
        record = logging.LogRecord(
            "dimos.plugins.runtime_log", logging.INFO, "", 0, name, (), None,
        )
        record.runtime_fields = _snapshot_fields({
            **metadata, "event": name, "path": path, "bytes": len(data),
        })
        return self._writer.offer((path, data, record))

    def _reject_diagnostic(self, reason):
        count = self._diagnostic_rejected.get(reason, 0)
        self._diagnostic_rejected[reason] = count + 1
        if count == 0:
            self.event("dimos.plugins.runtime_log", "diagnostic_artifact_rejected",
                       _snapshot_fields({"reason": reason}))

    def diagnostic_artifact(self, filename, data, metadata):
        if isinstance(data, str) and len(data) <= _DIAGNOSTIC_ITEM_LIMIT:
            data = data.encode("utf-8", errors="replace")
        with self._artifact_lock:
            if not isinstance(filename, str) or not _EVIDENCE_NAME.fullmatch(filename):
                self._reject_diagnostic("invalid_name")
                return None
            if not isinstance(data, bytes) or not 0 < len(data) <= _DIAGNOSTIC_ITEM_LIMIT:
                self._reject_diagnostic("item_size_or_type")
                return None
            if (
                self._diagnostic_files >= _DIAGNOSTIC_FILE_LIMIT
                or self._diagnostic_bytes + len(data) > _DIAGNOSTIC_LIMIT
            ):
                self._reject_diagnostic("archive_limit")
                return None
            path = f"diagnostics/{self._diagnostic_files + 1:03d}-{filename}"
            if not self._queue_artifact(path, data, metadata, "diagnostic_artifact_saved"):
                self._reject_diagnostic("queue_full")
                return None
            self._diagnostic_files += 1
            self._diagnostic_bytes += len(data)
            return path

    def _persist(self, item):
        if isinstance(item, tuple):
            path, data, record = item
            target = self._run_dir / path
            target.parent.mkdir(exist_ok=True)
            temporary = target.with_name("." + target.name + ".tmp")
            try:
                temporary.write_bytes(data)
                temporary.replace(target)
            finally:
                temporary.unlink(missing_ok=True)
        else:
            record = item
        parts = record.name.split(".")
        if len(parts) < 2:
            return
        component = parts[1]
        if component == "plugins" and len(parts) > 2:
            component = parts[2]
        if not MODULE_NAME.fullmatch(component):
            return
        if component not in self._files:
            self._directory.mkdir(exist_ok=True)
            self._files[component] = LogFile(self._directory / f"{component}.jsonl")
        line = (self._formatter.format(record) + "\n").encode("utf-8", errors="backslashreplace")
        self._files[component].write(line)
        if self._diagnostic_files or self._diagnostic_rejected:
            now = time.monotonic()
            if now >= self._next_diagnostic_health:
                self._next_diagnostic_health = now + 1.0
                self._report_diagnostics()

    def _report(self, dropped, errors):
        health = self._run_dir / "writer_health.json"
        temporary = health.with_suffix(".tmp")
        temporary.write_text(
            json.dumps({"dropped": dropped, "write_errors": errors}) + "\n",
            encoding="utf-8",
        )
        temporary.replace(health)

    def _report_diagnostics(self):
        with self._artifact_lock:
            summary = {
                "queued_files": self._diagnostic_files, "queued_bytes": self._diagnostic_bytes,
                "rejected": dict(self._diagnostic_rejected),
            }
        health = self._run_dir / "diagnostic_health.json"
        temporary = health.with_suffix(".tmp")
        temporary.write_text(json.dumps(summary) + "\n", encoding="utf-8")
        temporary.replace(health)

    def _finish(self, dropped, errors):
        for sink in self._files.values():
            try:
                sink.close()
            except OSError:
                errors += 1
        self._files.clear()
        if self._diagnostic_files or self._diagnostic_rejected:
            try:
                self._report_diagnostics()
            except OSError:
                errors += 1
        if dropped or errors:
            self._report(dropped, errors)

    def close(self):
        if not self._closed:
            super().close()
            self._writer.close()


@contextmanager
def capture(module, *, ros=False, base=None, metadata=None, evidence=None):
    """Capture optional diagnostics without waiting for file writes during operation."""
    global _active_components
    with ExitStack() as resources:
        try:
            session = _recording_session() if base is None else None
            if session is not None:
                base = session["source"]
                evidence = session.get("evidence", "off")
            run = create_run(module, base=base, ros=ros, metadata=metadata)
            if run is not None:
                stdout, stderr = (run / filename for filename in ("stdout.log", "stderr.log"))
                stdout.touch()
                stderr.touch()
                handler = _Components(run, evidence=evidence)
                resources.callback(handler.close)
        except (OSError, RuntimeError, TypeError, ValueError) as exc:
            print(f"Runtime logging unavailable: {exc}", file=sys.stderr)
            run = None
        if run is None:
            yield None
            return

        if ros:
            previous = os.environ.get("ROS_LOG_DIR")
            if previous is None:
                resources.callback(os.environ.pop, "ROS_LOG_DIR", None)
            else:
                resources.callback(os.environ.__setitem__, "ROS_LOG_DIR", previous)
            os.environ["ROS_LOG_DIR"] = str(run / "ros")

        native = ExitStack()
        try:
            _mirror_fd(native, sys.stdout, 1, stdout)
            _mirror_fd(native, sys.stderr, 2, stderr)
        except (OSError, ValueError):
            native.close()
            for stream, path, redirect in (
                (sys.stdout, stdout, redirect_stdout), (sys.stderr, stderr, redirect_stderr),
            ):
                target = ConsoleFile(path)
                try:
                    writer = BackgroundWriter(
                        target.write, target.close, capacity=64, name="console-log",
                    )
                except RuntimeError:
                    continue
                resources.callback(writer.close)
                resources.enter_context(redirect(_Mirror(stream, writer)))
        else:
            resources.enter_context(native)

        logger = logging.getLogger("dimos")
        logger.addHandler(handler)
        resources.callback(logger.removeHandler, handler)
        for name in _TASK_LOG_NAMESPACES:
            component_logger = logging.getLogger(name)
            resources.callback(component_logger.setLevel, component_logger.level)
            if component_logger.getEffectiveLevel() > logging.INFO:
                component_logger.setLevel(logging.INFO)

        previous_handler, _active_components = _active_components, handler
        try:
            yield run
        finally:
            _active_components = previous_handler
