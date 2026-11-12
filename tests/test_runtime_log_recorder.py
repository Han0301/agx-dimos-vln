"""Run the recorder and interrupt it with live, independently managed producers."""

import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
RECORD = """
import sys
from dimos.plugins.runtime_log.recorder import record
try:
    mode = sys.argv[3] if len(sys.argv) > 3 else None
    raise SystemExit(record(sys.argv[1], spool_root=sys.argv[2], mode=mode))
except RuntimeError as exc:
    raise SystemExit(str(exc)) from exc
"""


def _until(predicate):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.03)
    assert predicate()


def _read(path):
    return path.read_text() if path.exists() else ""


def _events(path, run=None):
    events = (json.loads(line) for line in _read(path).splitlines())
    return [event["event"] for event in events if not run or event.get("run") == run]


def _test_run(source, module, name):
    run = source / module / name
    (run / "modules").mkdir(parents=True)
    (run / "run.json").write_text(
        json.dumps(
            {"module": module, "pid": os.getpid(), "started_ns": time.monotonic_ns()}
        )
    )
    return run


def _record_line(message):
    return (
        json.dumps(
            {
                "time": "2026-11-11T12:00:00+08:00",
                "level": "INFO",
                "message": message,
                "task_id": "test_task",
            }
        )
        + "\n"
    ).encode()


def test_recording_saves_changed_files_and_ctrl_c_leaves_producer_running(tmp_path):
    output, spool = tmp_path / "log", tmp_path / "spool"
    recorder_command = [sys.executable, "-c", RECORD, str(output), str(spool)]
    recorder = subprocess.Popen(
        recorder_command, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.PIPE
    )
    producer = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"])
    try:
        marker = output / ".active-session"
        _until(marker.is_file)
        session = json.loads(marker.read_text())
        source = Path(session["source"])
        dest = output / session["session"]
        run = source / "test_module" / "run_1"
        (run / "ros").mkdir(parents=True)
        (run / "run.json").write_text(
            json.dumps(
                {
                    "module": "test_module",
                    "pid": producer.pid,
                    "started_ns": time.monotonic_ns(),
                    "started_at": "2026-11-11T12:00:00+08:00",
                }
            )
        )
        (run / "stdout.log").write_text("first output\n")
        (run / "stderr.log").write_text("warning\n")
        (run / "ros" / "native.log").write_text("ROS log\n")
        (run / "ignored.log").symlink_to(run / "stdout.log")

        duplicate = subprocess.run(
            recorder_command, cwd=ROOT, capture_output=True, timeout=5, check=False
        )
        assert duplicate.returncode == 1
        assert b"already active" in duplicate.stderr
        assert json.loads(marker.read_text())["pid"] == recorder.pid

        recorded_run = dest / "test_module" / "run_1"
        _until(lambda: _read(recorded_run / "stdout.log") == "first output\n")
        timing = dest / "timing.jsonl"
        _until(lambda: "first_output" in _events(timing))

        short = subprocess.Popen([sys.executable, "-c", "pass"])
        short.wait(timeout=5)
        camera = source / "cameras" / "short_run"
        camera.mkdir(parents=True)
        (camera / "run.json").write_text(
            json.dumps(
                {
                    "module": "cameras",
                    "pid": short.pid,
                    "started_ns": time.monotonic_ns(),
                    "started_at": "2026-11-11T12:00:00+08:00",
                }
            )
        )
        (camera / "d455.log").write_text("camera output\n")
        _until(lambda: "finished" in _events(timing, "cameras/short_run"))

        (run / "stdout.log").rename(run / "stdout.log.1")
        (run / "stdout.log").write_text("rotated output\n")
        _until(lambda: _read(recorded_run / "stdout.log") == "rotated output\n")

        with (run / "stdout.log").open("a") as stream:
            stream.write("last output\n")
        recorder.send_signal(signal.SIGINT)
        stdout, stderr = recorder.communicate(timeout=5)
        assert recorder.returncode == 0, stderr.decode()
        assert b"Runtime logs saved" in stdout
        assert b"started test_module/run_1" in stdout
        assert b"first_output cameras/short_run" in stdout
        assert b"finished cameras/short_run" in stdout
        assert producer.poll() is None
        assert not marker.exists()
        assert _read(recorded_run / "stdout.log") == "rotated output\nlast output\n"
        assert _read(recorded_run / "stdout.log.1") == "first output\n"
        assert _read(recorded_run / "stderr.log") == "warning\n"
        assert _read(recorded_run / "ros" / "native.log") == "ROS log\n"
        assert not (recorded_run / "ignored.log").exists()
        events = _events(timing)
        assert events[0] == "recording_started"
        assert events[-1] == "recording_stopped"
        assert _events(timing, "test_module/run_1") == ["started", "first_output"]
        assert _events(timing, "cameras/short_run") == [
            "started",
            "first_output",
            "finished",
        ]
        (run / "stdout.log").write_text("after stop\n")
        assert _read(recorded_run / "stdout.log") == "rotated output\nlast output\n"
    finally:
        for process in (recorder, producer):
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=5)


def test_recording_mirrors_new_complete_agent_and_tracker_metadata_only(tmp_path):
    output, spool = tmp_path / "log", tmp_path / "spool"
    console = tmp_path / "terminal"
    with console.open("w") as terminal:
        recorder = subprocess.Popen(
            [sys.executable, "-c", RECORD, str(output), str(spool), "bag"],
            cwd=ROOT,
            stdout=terminal,
            stderr=subprocess.PIPE,
        )
    try:
        marker = output / ".active-session"
        _until(marker.is_file)
        session = json.loads(marker.read_text())
        source = Path(session["source"])
        dest = output / session["session"]

        web = _test_run(source, "web_input", "run_1")
        (web / "stdout.log").write_text("raw_web_output\n")
        (web / "modules" / "other.jsonl").write_bytes(
            _record_line("unrelated_metadata")
        )
        active = web / "modules" / "llm.jsonl"
        first_record = _record_line("agent_call_finished")
        active.write_bytes(first_record[:24])
        archived = dest / "web_input" / "run_1" / "modules" / "llm.jsonl"
        _until(lambda: len(_read(archived)) == 24)
        assert "agent_call_finished" not in _read(console)

        with active.open("ab") as stream:
            stream.write(first_record[24:])
        _until(lambda: "llm INFO task=test_task agent_call_finished" in _read(console))
        active.rename(active.with_suffix(".jsonl.1"))
        active.write_bytes(_record_line("second_agent_call"))
        _until(lambda: "second_agent_call" in _read(console))

        tracker = _test_run(source, "path_tracker", "run_2")
        (tracker / "stderr.log").write_text("raw_tracker_output\n")
        (tracker / "modules" / "path_tracker.jsonl").write_bytes(
            _record_line("velocity_command")
        )
        _until(
            lambda: (
                "path_tracker INFO task=test_task velocity_command" in _read(console)
            )
        )
        recorder.send_signal(signal.SIGINT)
        _, stderr = recorder.communicate(timeout=5)
        assert recorder.returncode == 0, stderr.decode()
        mirrored = _read(console)
        assert mirrored.count("agent_call_finished") == 1
        assert mirrored.count("second_agent_call") == 1
        assert mirrored.count("velocity_command") == 1
        assert "unrelated_metadata" not in mirrored
        assert "raw_web_output" not in mirrored
        assert "raw_tracker_output" not in mirrored
        assert (dest / "web_input/run_1/stdout.log").read_text() == "raw_web_output\n"
        assert (
            dest / "path_tracker/run_2/stderr.log"
        ).read_text() == "raw_tracker_output\n"
        assert (
            json.loads((dest / "timing.jsonl").read_text().splitlines()[0])["mode"]
            == "bag"
        )
    finally:
        if recorder.poll() is None:
            recorder.terminate()
            recorder.wait(timeout=5)


@pytest.mark.parametrize("extra_args,expected", [
    ([], "failures"),
    (["--evidence", "off"], "off"),
])
def test_recorder_cli_passes_evidence_mode(tmp_path, monkeypatch, extra_args, expected):
    from dimos.plugins.runtime_log import recorder
    from dimos.plugins.runtime_log.__main__ import main

    def record(base, *, mode, evidence):
        assert base == tmp_path
        assert mode == "live"
        assert evidence == expected
        return 0

    monkeypatch.setattr(recorder, "record", record)
    monkeypatch.setattr(
        sys, "argv", ["runtime_log", "record", "--root", str(tmp_path), "--mode", "live", *extra_args],
    )
    assert main() == 0


def test_recorder_stop_preserves_registered_producer_and_default_evidence(tmp_path):
    output, spool, resume = (tmp_path / name for name in ("log", "spool", "resume"))
    recorder = subprocess.Popen(
        [
            sys.executable, "-c",
            "import sys; from dimos.plugins.runtime_log.recorder import record; "
            "record(sys.argv[1], spool_root=sys.argv[2])",
            str(output), str(spool),
        ],
        cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    )
    producer = None
    try:
        marker = output / ".active-session"
        _until(marker.is_file)
        session = json.loads(marker.read_text())
        dest = output / session["session"]
        assert session["evidence"] == "failures"
        assert json.loads((dest / "timing.jsonl").read_text().splitlines()[0])["evidence"] == "failures"
        producer = subprocess.Popen(
            [sys.executable, "-c", """
import logging
import sys
import time
from pathlib import Path
from dimos.plugins import runtime_log as logs

logs.SPOOL_ROOT = Path(sys.argv[1])
resume = Path(sys.argv[2])
logger = logging.getLogger("dimos.vln_task")
with logs.capture("web_input"):
    with logs.task_context("demo"):
        path = logs.artifact("failed-frame.jpg", b"demo frame")
        response = logs.artifact("failed-response.txt", "demo model response")
        logs.event(logger, "before_stop", evidence_path=path, response_path=response)
        deadline = time.monotonic() + 8
        while not resume.exists() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert resume.exists()
        logs.event(logger, "after_stop")
        print("still working")
""", str(spool), str(resume)],
            env={**os.environ, "DIMOS_LOG_ROOT": str(output)}, cwd=ROOT,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        )
        _until(lambda: any(dest.glob("web_input/*/modules/vln_task.jsonl")))
        (run,) = (dest / "web_input").iterdir()
        event_path = run / "modules/vln_task.jsonl"
        _until(lambda: "before_stop" in _read(event_path))
        before = json.loads(_read(event_path))
        _until(lambda: (run / before["evidence_path"]).is_file())
        recorder.send_signal(signal.SIGINT)
        _, recorder_stderr = recorder.communicate(timeout=5)
        assert recorder.returncode == 0, recorder_stderr.decode()
        assert producer.poll() is None
        resume.touch()
        stdout, stderr = producer.communicate(timeout=5)
        assert producer.returncode == 0, stderr.decode()
        assert stdout == b"still working\n"
        assert "after_stop" not in _read(event_path)
        assert (run / before["evidence_path"]).read_bytes() == b"demo frame"
        assert (run / before["response_path"]).read_text() == "demo model response"
        assert before["task_id"] == "demo"
    finally:
        for process in (recorder, producer):
            if process is not None and process.poll() is None:
                process.terminate()
                process.wait(timeout=5)
