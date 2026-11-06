"""Full debug recording session used by ``dimos --debug_mode true``."""

from __future__ import annotations

import atexit
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from typing import Any

from dimos.utils.runtime_paths import DIMOS_RUN_ROOT


def _proc_start_ticks(pid: int) -> int:
    raw = Path("/proc", str(pid), "stat").read_text()
    return int(raw[raw.rfind(")") + 2 :].split()[19])


def write_debug_mark(
    message: str,
    *,
    log_root: str | Path | None = None,
    session_id: str = "",
) -> Path:
    root = Path(log_root).expanduser() if log_root else DIMOS_RUN_ROOT
    sessions: list[dict[str, Any]] = []
    manifests = (
        root.glob("*/logs/session/session.json")
        if root == DIMOS_RUN_ROOT or log_root is not None
        else root.glob("*/session/session.json")
    )
    for manifest in manifests:
        try:
            data = json.loads(manifest.read_text())
            if session_id and data["run_id"] != session_id:
                continue
            if _proc_start_ticks(int(data["recorder_pid"])) == data["recorder_start_ticks"]:
                sessions.append(data)
        except (OSError, ValueError, KeyError, IndexError):
            continue
    if len(sessions) != 1:
        raise RuntimeError("需要恰好一个活动 debug 会话；多个会话时请指定 --session")
    marker = {
        "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
        "event": "marker",
        "text": message,
    }
    marker_file = Path(sessions[0]["marker_file"])
    with marker_file.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(marker, ensure_ascii=False) + "\n")
    return marker_file


class DebugSession:
    def __init__(self, run_id: str, *, log_dir: Path, poll_hz: float = 50.0) -> None:
        self.run_id = run_id
        self.poll_hz = poll_hz
        self.log_dir = log_dir.expanduser()
        self.session_dir = self.log_dir / "session"
        self.terminal_log = self.log_dir / "dimos_terminal.log"
        self.timing_log = self.log_dir / "terminal.timing"
        self.trace_log = self.log_dir / "drive_trace.jsonl"
        self.marker_log = self.log_dir / "debug_marks.jsonl"
        self.status_log = self.session_dir / "recorder_status.log"
        self._lock = threading.Lock()
        self._terminal_stream: Any = None
        self._timing_fd: int | None = None
        self._started_monotonic = 0.0
        self._recorder: subprocess.Popen[bytes] | None = None
        self._status_stream: Any = None
        self._saved_fds: dict[int, int] = {}
        self._reader_threads: list[threading.Thread] = []
        self._closing = False
        self._closed = False

    def start(self) -> None:
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.session_dir.mkdir(exist_ok=True)
        self.marker_log.touch(mode=0o600)
        self._terminal_stream = self.terminal_log.open("wb", buffering=0)
        self._timing_fd = os.open(self.timing_log, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        self._started_monotonic = time.monotonic()

        self._status_stream = self.status_log.open("wb")
        self._recorder = subprocess.Popen(
            [
                sys.executable,
                "-m",
                "dimos.utils.debug_trace",
                "--output",
                str(self.trace_log),
                "--poll-hz",
                str(self.poll_hz),
                "--system",
                "--process-root",
                str(os.getpid()),
            ],
            stdout=self._status_stream,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        time.sleep(0.15)
        if self._recorder.poll() is not None:
            raise RuntimeError(f"debug recorder failed; see {self.status_log}")

        manifest = {
            "run_id": self.run_id,
            "main_pid": os.getpid(),
            "recorder_pid": self._recorder.pid,
            "recorder_start_ticks": _proc_start_ticks(self._recorder.pid),
            "marker_file": str(self.marker_log),
        }
        (self.session_dir / "session.json").write_text(json.dumps(manifest, indent=2))
        (self.session_dir / "session.bashrc").write_text(
            "debug_mark() { dimos debug-mark \"$*\"; }\n"
            f"export DIMOS_DEBUG_SESSION={self.run_id}\n"
        )

        for fd, stream_name in ((1, "stdout"), (2, "stderr")):
            read_fd, write_fd = os.pipe()
            self._saved_fds[fd] = os.dup(fd)
            os.dup2(write_fd, fd)
            os.close(write_fd)
            thread = threading.Thread(
                target=self._tee_fd,
                args=(read_fd, self._saved_fds[fd], stream_name),
                daemon=True,
                name=f"debug-tee-{stream_name}",
            )
            thread.start()
            self._reader_threads.append(thread)
        for stream in (sys.stdout, sys.stderr):
            try:
                stream.reconfigure(line_buffering=True, write_through=True)
            except (AttributeError, ValueError):
                pass
        atexit.register(self.close)
        print(f"Debug session: {self.log_dir}", flush=True)

    def _tee_fd(self, read_fd: int, terminal_fd: int, stream_name: str) -> None:
        try:
            while data := os.read(read_fd, 65536):
                os.write(terminal_fd, data)
                self._record_output(stream_name, data)
        finally:
            os.close(read_fd)

    def _record_output(self, stream_name: str, data: bytes) -> None:
        with self._lock:
            if self._closed or self._terminal_stream is None or self._timing_fd is None:
                return
            self._terminal_stream.write(data)
            self._terminal_stream.flush()
            elapsed = time.monotonic() - self._started_monotonic
            header = f"{elapsed:010.6f} {len(data)} {stream_name}\n".encode()
            os.write(self._timing_fd, header)
            os.write(self._timing_fd, data)

    def close(self) -> None:
        with self._lock:
            if self._closing or self._closed:
                return
            self._closing = True
        for stream in (sys.stdout, sys.stderr):
            try:
                stream.flush()
            except Exception:
                pass
        for fd, saved in self._saved_fds.items():
            os.dup2(saved, fd)
        for thread in self._reader_threads:
            thread.join(timeout=1.0)
        self._reader_threads.clear()
        for saved in self._saved_fds.values():
            os.close(saved)
        self._saved_fds.clear()
        with self._lock:
            self._closed = True
        if self._recorder is not None and self._recorder.poll() is None:
            self._recorder.send_signal(signal.SIGTERM)
            try:
                self._recorder.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._recorder.kill()
                self._recorder.wait()
        if self._terminal_stream is not None:
            self._terminal_stream.close()
            self._terminal_stream = None
        if self._timing_fd is not None:
            os.close(self._timing_fd)
            self._timing_fd = None
        if self._status_stream is not None:
            self._status_stream.close()
            self._status_stream = None
        print(
            "Debug session stopped:\n"
            f"  terminal: {self.terminal_log}\n"
            f"  telemetry: {self.trace_log}\n"
            f"  timing: {self.timing_log}\n"
            f"  status: {self.status_log}",
            flush=True,
        )


def start_debug_session(enabled: bool, run_id: str, log_dir: Path) -> DebugSession | None:
    if not enabled:
        return None
    session = DebugSession(run_id, log_dir=log_dir)
    session.start()
    return session
