import json
import logging
import threading


def test_structured_event_keeps_producer_context(tmp_path):
    from dimos.plugins.runtime_log import capture, event, event_context, task_context

    logger = logging.getLogger("dimos.plugins.vln_task")
    with capture("web_input", base=tmp_path) as run:
        with task_context("task-42"), event_context(purpose="first_det", det_id="d1"):
            event(logger, "detected", bbox_2d=[1, 2, 3, 4], purpose="override")
            logger.info("regular record")

    detected, regular = [
        json.loads(line)
        for line in (run / "modules/vln_task.jsonl").read_text().splitlines()
    ]
    assert detected["event"] == "detected"
    assert detected["task_id"] == "task-42"
    assert detected["purpose"] == "override"
    assert detected["det_id"] == "d1"
    assert detected["bbox_2d"] == [1, 2, 3, 4]
    assert detected["monotonic_ns"] > 0
    assert regular["task_id"] == "task-42"


def test_slow_disk_and_full_queue_leave_producer_running(tmp_path, monkeypatch):
    from dimos.plugins import runtime_log

    started = threading.Event()
    unblock = threading.Event()
    producer_done = threading.Event()
    real_write = runtime_log.LogFile.write

    def slow_file(*args, **kwargs):
        started.set()
        if not unblock.wait(4):
            raise OSError("test disk unavailable")
        return real_write(*args, **kwargs)

    monkeypatch.setattr(runtime_log.LogFile, "write", slow_file)
    monkeypatch.setattr(runtime_log, "EVENT_QUEUE_SIZE", 4)
    logger = logging.getLogger("dimos.plugins.path_tracker")
    try:
        with runtime_log.capture("web_input", base=tmp_path) as run:
            runtime_log.event(logger, "first_write")
            assert started.wait(1)

            def produce():
                for _ in range(100):
                    logger.info("control loop progress")
                    runtime_log.event(logger, "velocity", linear_x=0.4)
                producer_done.set()

            worker = threading.Thread(target=produce)
            worker.start()
            try:
                assert producer_done.wait(1)
            finally:
                unblock.set()
                worker.join(2)
    finally:
        unblock.set()

    health = json.loads((run / "writer_health.json").read_text())
    assert health["dropped"] > 0


def test_unwritable_capture_and_writer_failure_are_nonfatal(tmp_path, monkeypatch):
    from dimos.plugins import runtime_log

    invalid_root = tmp_path / "file"
    invalid_root.touch()
    with runtime_log.capture("web_input", base=invalid_root) as run:
        assert run is None
        assert not runtime_log.recording_enabled()
        assert runtime_log.event(logging.getLogger("dimos.plugins.vln_task"), "ignored") is None

    def disk_full(*args, **kwargs):
        raise OSError("disk full")

    monkeypatch.setattr(runtime_log.LogFile, "write", disk_full)
    with runtime_log.capture("web_input", base=tmp_path) as run:
        runtime_log.event(logging.getLogger("dimos.plugins.vln_task"), "detected")
        assert runtime_log.recording_enabled()
    health = json.loads((run / "writer_health.json").read_text())
    assert health["write_errors"] == 1


def test_evidence_is_opt_in_and_bounded(tmp_path):
    from dimos.plugins.runtime_log import artifact, capture

    with capture("web_input", base=tmp_path):
        assert artifact("camera.jpg", b"image") is None

    with capture("web_input", base=tmp_path, evidence="failures") as run:
        assert artifact("../escape", b"no") is None
        assert artifact("camera.jpg", bytes(1024 * 1024 + 1)) is None
        reference = artifact("camera.jpg", b"image", rgb_stamp_ns=123)
        assert reference is not None

    assert (run / reference).read_bytes() == b"image"
    index = json.loads((run / "modules/runtime_log.jsonl").read_text())
    assert index["event"] == "artifact_saved"
    assert index["rgb_stamp_ns"] == 123
    assert index["path"] == reference


def test_native_relay_keeps_forwarding_when_its_disk_open_is_blocked(tmp_path):
    import os
    import subprocess
    import sys

    target = tmp_path / "blocked"
    os.mkfifo(target)
    data = b"control output\n" * 200_000
    result = subprocess.run(
        [sys.executable, "-m", "dimos.plugins.runtime_log.relay", str(target)],
        input=data, capture_output=True, timeout=4,
    )
    assert result.returncode == 0
    assert result.stdout == data
    assert result.stderr == b""
