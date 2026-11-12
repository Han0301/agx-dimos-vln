"""Collect logs from modules launched during an explicitly recorded session."""

import errno
import fcntl
import json
import os
import signal
import stat
import threading
import time
from datetime import datetime
from pathlib import Path
from tempfile import mkdtemp

from . import (
    DEFAULT_EVIDENCE, LOCAL_TIME, SCHEMA_VERSION, SPOOL_ROOT,
    environment_metadata, get_start_ticks, log_root,
)

POLL_INTERVAL = 0.25
LIVE_COMPONENTS = {
    "web_input": {
        "web_input",
        "vln_task",
        "llm",
        "vlm",
        "active_search",
        "xnavdp",
        "path_tracker",
    },
    "path_tracker": {"path_tracker"},
    "body": {"robot"},
}


def _timestamp():
    return datetime.now(LOCAL_TIME).isoformat(timespec="milliseconds")


class _Collector:
    def __init__(self, source, output, timing):
        self.source = source
        self.output = output
        self.timing = timing
        self.copied = {}
        self.runs = {}
        self.first_output = set()
        self.finished = set()
        self.partial_lines = {}

    def event(self, name, **fields):
        row = {"event": name, "observed_at": _timestamp(), **fields}
        self.timing.write(json.dumps(row, ensure_ascii=False) + "\n")
        self.timing.flush()
        if name in {"started", "first_output", "finished"}:
            message = f"[{row['observed_at']}] {name} {fields['run']}"
            if "pid" in fields:
                message += f" pid={fields['pid']}"
            if "elapsed_ms" in fields:
                message += f" observed_elapsed_ms={fields['elapsed_ms']:.1f}"
            print(message, flush=True)

    @staticmethod
    def _live_component(relative):
        if len(relative.parts) != 4 or relative.parts[2] != "modules":
            return None
        if relative.suffix != ".jsonl":
            return None
        if relative.stem in LIVE_COMPONENTS.get(relative.parts[0], ()):
            return relative.stem
        return None

    def _show_records(self, relative, component, chunk):
        lines = (self.partial_lines.get(relative, b"") + chunk).split(b"\n")
        self.partial_lines[relative] = lines.pop()
        for line in lines:
            try:
                event = json.loads(line)
            except (ValueError, UnicodeError):
                continue
            if not isinstance(event, dict) or not all(
                isinstance(event.get(key), str) for key in ("time", "level", "message")
            ):
                continue
            if "event" in event and event["event"] not in {
                "task_submitted", "task_finished", "stop_requested", "tracker_arrival",
                "body_command_forwarded", "body_command_failed", "body_stop_observed",
                "body_stop_observation_expired",
            }:
                continue
            task_id = event.get("task_id")
            task = f" task={task_id}" if isinstance(task_id, str) and task_id else ""
            message = f"[{event['time']}] {component} {event['level']}{task} {event['message']}"
            print(
                "".join(char if char.isprintable() else " " for char in message),
                flush=True,
            )

    def _copy(self, source):
        relative = source.relative_to(self.source)
        component = self._live_component(relative)
        try:
            fd = os.open(source, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
        except OSError as exc:
            if exc.errno in {errno.ENOENT, errno.ENOTDIR, errno.ELOOP}:
                return 0
            raise
        with os.fdopen(fd, "rb") as input_file:
            info = os.fstat(fd)
            if not stat.S_ISREG(info.st_mode):
                return 0
            identity = (info.st_dev, info.st_ino)
            previous_id, offset = self.copied.get(relative, (None, 0))
            replace = previous_id != identity or info.st_size < offset
            if replace:
                offset = 0
                self.partial_lines.pop(relative, None)
            elif info.st_size == offset:
                return offset
            target = self.output / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            input_file.seek(offset)
            with target.open("wb" if replace else "ab") as output_file:
                # Limit each pass to the size observed before copying.
                while offset < info.st_size:
                    chunk = input_file.read(min(65536, info.st_size - offset))
                    if not chunk:
                        break
                    output_file.write(chunk)
                    offset += len(chunk)
                    if component is not None:
                        self._show_records(relative, component, chunk)
            self.copied[relative] = (identity, offset)
            return offset

    def sync(self):
        for metadata in self.source.glob("*/*/run.json"):
            run = metadata.parent
            if metadata.is_symlink() or run.is_symlink() or run.parent.is_symlink():
                continue
            relative = run.relative_to(self.source).as_posix()
            if relative not in self.runs:
                try:
                    data = json.loads(metadata.read_text())
                except (OSError, ValueError):
                    continue
                if (
                    not isinstance(data, dict)
                    or not isinstance(data.get("module"), str)
                    or type(data.get("pid")) is not int
                    or data["pid"] <= 0
                    or type(data.get("started_ns")) is not int
                ):
                    continue
                self.runs[relative] = data
                self.event(
                    "started",
                    module=data["module"],
                    run=relative,
                    pid=data["pid"],
                    started_at=data.get("started_at"),
                    started_ns=data["started_ns"],
                    metadata=data.get("metadata", {}),
                )
            data = self.runs[relative]
            output_streams = []
            console_files = {"stdout.log", "stderr.log"}
            if data["module"] == "cameras":
                console_files.update(("d455.log", "x3.log"))
            for directory, _, files in os.walk(run, followlinks=False):
                for filename in files:
                    if filename.endswith(".tmp"):
                        continue
                    path = Path(directory) / filename
                    if path.is_symlink():
                        continue
                    size = self._copy(path)
                    if path.parent == run and filename in console_files and size:
                        output_streams.append(filename)
            if output_streams and relative not in self.first_output:
                self.first_output.add(relative)
                self.event(
                    "first_output",
                    module=data["module"],
                    run=relative,
                    streams=sorted(output_streams),
                    elapsed_ms=(time.monotonic_ns() - data["started_ns"]) / 1e6,
                )
            if relative in self.finished:
                continue
            ticks = get_start_ticks(data["pid"])
            expected = data.get("start_ticks")
            if ticks is None or (expected is not None and str(expected) != ticks):
                self.finished.add(relative)
                self.event(
                    "finished",
                    module=data["module"],
                    run=relative,
                    pid=data["pid"],
                    elapsed_ms=(time.monotonic_ns() - data["started_ns"]) / 1e6,
                )


def record(base=None, *, spool_root=None, mode=None, evidence=DEFAULT_EVIDENCE):
    """Record until interrupted, without sending signals to producer modules."""
    base = log_root(base)
    base.mkdir(parents=True, exist_ok=True)
    with (base / ".record.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError(f"A log recording is already active in {base}") from exc

        prefix = datetime.now(LOCAL_TIME).strftime("session_%Y%m%d_%H%M%S_%f_")
        output = Path(mkdtemp(prefix=f"{prefix}{os.getpid()}_", dir=base))
        source = Path(spool_root or SPOOL_ROOT).resolve() / output.name
        source.mkdir(parents=True, mode=0o700)
        marker = base / ".active-session"
        temporary = base / f".active-session-{os.getpid()}"
        stopped = threading.Event()
        signals = (signal.SIGINT, signal.SIGTERM)
        handlers = {
            sig: signal.signal(sig, lambda *_: stopped.set()) for sig in signals
        }
        try:
            with (output / "timing.jsonl").open("w", encoding="utf-8") as timing:
                collector = _Collector(source, output, timing)
                collector.event(
                    "recording_started", pid=os.getpid(), mode=mode, evidence=evidence,
                    schema_version=SCHEMA_VERSION, monotonic_ns=time.monotonic_ns(),
                    metadata=environment_metadata(),
                )
                temporary.write_text(
                    json.dumps(
                        {
                            "source": str(source),
                            "pid": os.getpid(),
                            "session": output.name,
                            "evidence": evidence,
                            "start_ticks": get_start_ticks(os.getpid()),
                        }
                    ),
                    encoding="utf-8",
                )
                temporary.replace(marker)
                try:
                    print(f"Recording runtime logs: {output}", flush=True)
                    while not stopped.wait(POLL_INTERVAL):
                        collector.sync()
                finally:
                    marker.unlink(missing_ok=True)
                    collector.sync()
                    collector.event(
                        "recording_stopped", monotonic_ns=time.monotonic_ns(),
                        run_count=len(collector.runs),
                        modules=sorted({data["module"] for data in collector.runs.values()}),
                    )
                print(f"Runtime logs saved: {output}", flush=True)
        finally:
            temporary.unlink(missing_ok=True)
            for sig, handler in handlers.items():
                signal.signal(sig, handler)
    return 0
