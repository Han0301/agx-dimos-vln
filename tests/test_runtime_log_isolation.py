import json
import logging
import threading


def test_structured_event_keeps_producer_context(tmp_path):
    from dimos.plugins.runtime_log import capture, event, event_context, task_context

    logger = logging.getLogger("dimos.vln_task")
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
        assert runtime_log.event(logging.getLogger("dimos.vln_task"), "ignored") is None

    def disk_full(*args, **kwargs):
        raise OSError("disk full")

    monkeypatch.setattr(runtime_log.LogFile, "write", disk_full)
    with runtime_log.capture("web_input", base=tmp_path) as run:
        runtime_log.event(logging.getLogger("dimos.vln_task"), "detected")
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


def test_normal_capture_archives_replay_payload_separately_from_failure_evidence(tmp_path, monkeypatch):
    from dimos.plugins import runtime_log as logs

    monkeypatch.setattr(logs, "_DIAGNOSTIC_FILE_LIMIT", 1)
    with logs.capture("web_input", base=tmp_path) as run:
        with logs.task_context("task-a"):
            path = logs.diagnostic_artifact("depth.png", b"depth", observation_id="obs-a")
            assert path is not None
            assert logs.diagnostic_artifact("next.png", b"next") is None
            assert logs.artifact("failure.jpg", b"failure") is None

    assert (run / path).read_bytes() == b"depth"
    health = json.loads((run / "diagnostic_health.json").read_text())
    assert health == {"queued_files": 1, "queued_bytes": 5, "rejected": {"archive_limit": 1}}
    records = [
        json.loads(line) for line in (run / "modules/runtime_log.jsonl").read_text().splitlines()
    ]
    assert any(
        record.get("event") == "diagnostic_artifact_saved"
        and record.get("task_id") == "task-a" and record.get("observation_id") == "obs-a"
        and record.get("path") == path for record in records
    )


def test_run_metadata_is_lazy_and_only_loaded_during_capture(tmp_path, monkeypatch):
    from dimos.plugins import runtime_log as logs

    monkeypatch.delenv("DIMOS_LOG_ROOT", raising=False)
    monkeypatch.setattr(logs, "_recording_session", lambda: None)
    called = []

    def metadata():
        called.append(True)
        return {"probe": "captured"}

    with logs.capture("web_input", metadata=metadata) as run:
        assert run is None
    assert not called

    with logs.capture("web_input", base=tmp_path, metadata=metadata) as run:
        assert json.loads((run / "run.json").read_text())["metadata"]["probe"] == "captured"
    assert called == [True]


def test_run_provenance_preserves_safe_parameters_and_redacts_credentials(tmp_path, monkeypatch):
    from dimos.vln_task.diagnostic_metadata import diagnostics_metadata

    simple = tmp_path / "dimos_simple"
    weights = simple / "data/models_clip/model.onnx"
    weights.parent.mkdir(parents=True)
    weights.write_bytes(b"sample weights")
    monkeypatch.setenv("DIMOS_SIMPLE_ROOT", str(simple))
    effective = {
        "exploration_memory": True,
        "profile": {
            "api_key": "private",
            "endpoint": "https://user:password@example.org/?token=private",
            "max_new_tokens": 128,
        },
    }
    metadata = diagnostics_metadata(tmp_path / "dimos_mininum", effective)

    profile = metadata["effective_config"]["profile"]
    assert profile["max_new_tokens"] == 128
    assert profile["api_key"] == "[redacted]"
    assert "private" not in json.dumps(metadata)
    assert "user:password" not in json.dumps(metadata)
    assert metadata["checkouts"]["simple"]["root"] == str(simple)
    assert metadata["spatial_memory_model"]["size_bytes"] == len(b"sample weights")


def test_inactive_capture_creates_no_writer_or_archive(monkeypatch):
    from dimos.plugins import runtime_log as logs

    monkeypatch.delenv("DIMOS_LOG_ROOT", raising=False)
    monkeypatch.setattr(logs, "_active_components", None)
    monkeypatch.setattr(logs, "_recording_session", lambda: None)

    def unexpected(*args, **kwargs):
        raise AssertionError("inactive capture started a writer")

    monkeypatch.setattr(logs, "BackgroundWriter", unexpected)
    with logs.capture("web_input", metadata=unexpected) as run:
        assert run is None
        assert not logs.recording_enabled()
        assert logs.diagnostic_artifact("test.jpg", b"diagnostic") is None
