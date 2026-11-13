import io
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from dimos.plugins import spatial_memory as simple_memory


@pytest.fixture
def worker_root(monkeypatch, tmp_path):
    worker = tmp_path / "worker.py"
    worker.write_text('''import base64
import json
import sys
import time


def send(result):
    print(json.dumps({"ok": True, "result": result}), flush=True)


send(None)
pose = None
frame_id = "frame-test"
print("memory worker log", file=sys.stderr, flush=True)
for line in sys.stdin:
    request = json.loads(line)
    if request_id := request.get("memory_request_id"):
        print(f"worker received {request['action']} {request_id}", file=sys.stderr, flush=True)
    if request["action"] == "record":
        jpeg = base64.b64decode(request["jpeg"], validate=True)
        if pose is None:
            pose = request["pose"]
            send({"stored": True, "memory_id": frame_id, "image_width": 2, "image_height": 2})
        else:
            send({"stored": False, "reason": "distance_below_minimum", "distance_m": 0.0})
    else:
        if request["text"] == "slow":
            time.sleep(5)
        send([{"id": frame_id, "metadata": [{"pos_x": pose[0], "pos_y": pose[1], "rot_z": 0.25}], "distance": 0.0},
              {"metadata": [{"pos_x": 0}], "distance": 0.4}] if pose else [])
''')
    interpreter = tmp_path / ".venv/bin/python"
    interpreter.parent.mkdir(parents=True)
    interpreter.symlink_to(sys.executable)
    monkeypatch.setattr(simple_memory, "_WORKER_PATH", worker)
    monkeypatch.setenv("DIMOS_SIMPLE_ROOT", str(tmp_path))
    return tmp_path


def record_frame(memory, frame):
    p, q = frame.odom.pose.position, frame.odom.pose.orientation
    return memory.record(
        frame.jpeg, (p.x, p.y, p.z, q.x, q.y, q.z, q.w), lambda: 4.0,
        min_distance_m=0.05, min_yaw_delta_deg=15.0, min_interval_s=1.0,
        stamp_ns=getattr(frame, "stamp_ns", None),
        observation_id=getattr(frame, "observation_id", None),
    )


def test_memory_records_observation_and_returns_valid_query_hits(worker_root):
    pose = SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0, z=3.0),
                           orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
    frame = SimpleNamespace(jpeg=b"abc", odom=SimpleNamespace(pose=pose))
    memory = simple_memory.SimpleMemory()
    try:
        assert memory.query("box", lambda: 4.0) == []
        assert record_frame(memory, frame)
        assert memory.query("box", lambda: 4.0) == [(1.0, 2.0, 0.25, 1.0)]
        memory.reset()
        assert memory.query("box", lambda: 4.0) == []
    finally:
        memory.close()


def test_memory_query_timeout_allows_clean_restart(worker_root, monkeypatch):
    monkeypatch.setattr(simple_memory, "_OP_TIMEOUT_S", 0.2)
    memory = simple_memory.SimpleMemory()
    try:
        with pytest.raises(simple_memory.SimpleMemoryUnavailable, match="timed out"):
            memory.query("slow", lambda: 4.0)
        assert memory.query("box", lambda: 4.0) == []
    finally:
        memory.close()


def test_memory_preserves_task_cancellation(worker_root):
    from dimos.vln_task.agent import TaskCancelled

    def cancelled():
        raise TaskCancelled("stop")

    memory = simple_memory.SimpleMemory()
    try:
        with pytest.raises(TaskCancelled, match="stop"):
            memory.query("box", cancelled)
    finally:
        memory.close()


def test_stored_memory_image_and_lookup_share_worker_id(worker_root, monkeypatch):
    from PIL import Image

    from dimos.plugins.runtime_log import capture, task_context

    archive = worker_root / "archive"
    monkeypatch.setenv("DIMOS_LOG_ROOT", str(archive))
    image = io.BytesIO()
    Image.new("RGB", (2, 2)).save(image, "JPEG")
    jpeg = image.getvalue()
    pose = SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0, z=3.0),
                           orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
    frame = SimpleNamespace(jpeg=jpeg, stamp_ns=123, observation_id="observation-1",
                            odom=SimpleNamespace(pose=pose))

    memory = simple_memory.SimpleMemory()
    with capture("web_input", base=archive) as web_run, task_context("task-1"):
        try:
            assert record_frame(memory, frame)
            assert not record_frame(memory, frame)
            hits = memory.query_hits("box", lambda: 4.0)
            assert hits[0].memory_id == "frame-test"
        finally:
            memory.close()

    rows = [json.loads(line) for line in (web_run / "modules/spatial_memory.jsonl").read_text().splitlines()]
    stored, skipped = [row for row in rows if row.get("event") == "simple_memory_sample"]
    assert stored["memory_id"] == hits[0].memory_id
    assert stored["observation_id"] == "observation-1"
    assert stored["source_stamp_ns"] == 123
    assert stored["task_id"] == "task-1"
    assert (stored["image_width"], stored["image_height"], stored["image_bytes"]) == (2, 2, len(jpeg))
    image_path = archive / stored["image_path"]
    assert image_path.read_bytes() == jpeg
    worker_stderr = image_path.parent.parent.joinpath("stderr.log").read_text()
    assert worker_stderr.startswith("memory worker log\n")
    assert f"worker received record {stored['memory_request_id']}" in worker_stderr
    assert len(list(image_path.parent.glob("*.jpg"))) == 1
    assert skipped["reason"] == "distance_below_minimum"
    lookup = next(row for row in rows if row.get("event") == "simple_memory_lookup")
    assert lookup["candidate_ids"] == ["frame-test"]
    assert f"worker received query {lookup['memory_request_id']}" in worker_stderr
    assert lookup["memory_request_id"] != stored["memory_request_id"]
    assert lookup["query_text"] == "box"
    assert lookup["returned_candidates"] == 2
    assert lookup["candidate_details"] == [
        {"rank": 1, "memory_id": "frame-test", "xy": [1.0, 2.0], "yaw": 0.25, "similarity": 1.0},
    ]
    assert [row["event"] for row in rows if row.get("event", "").startswith("simple_memory_lookup")] == [
        "simple_memory_lookup_started", "simple_memory_lookup",
    ]
    lookup_start = next(row for row in rows if row.get("event") == "simple_memory_lookup_started")
    assert lookup_start["memory_request_id"] == lookup["memory_request_id"]
    ready = next(row for row in rows if row.get("event") == "simple_memory_worker" and row["phase"] == "ready")
    assert archive / ready["stderr_path"] == image_path.parent.parent / "stderr.log"


def test_memory_worker_timeout_is_distinct_from_a_lookup_miss(worker_root, monkeypatch):
    from dimos.plugins.runtime_log import capture, event_context

    archive = worker_root / "archive"
    monkeypatch.setenv("DIMOS_LOG_ROOT", str(archive))
    monkeypatch.setattr(simple_memory, "_OP_TIMEOUT_S", 0.2)
    memory = simple_memory.SimpleMemory()
    with capture("web_input", base=archive) as web_run, event_context(memory_lookup_id="lookup-1"):
        try:
            with pytest.raises(simple_memory.SimpleMemoryUnavailable, match="timed out"):
                memory.query_hits("slow", lambda: 4.0)
        finally:
            memory.close()

    rows = [json.loads(line) for line in (web_run / "modules/spatial_memory.jsonl").read_text().splitlines()]
    lookup = next(row for row in rows if row.get("event") == "simple_memory_lookup")
    assert lookup["memory_lookup_id"] == "lookup-1"
    assert (lookup["outcome"], lookup["reason"]) == ("error", "worker_timeout")
    stop = next(row for row in rows if row.get("event") == "simple_memory_worker" and row["phase"] == "stopped")
    assert stop["reason"] == "worker_timeout"


def test_invalid_pose_is_recorded_in_log_without_nonfinite_json(worker_root, monkeypatch):
    from dimos.plugins.runtime_log import capture

    archive = worker_root / "archive"
    monkeypatch.setenv("DIMOS_LOG_ROOT", str(archive))
    pose = SimpleNamespace(position=SimpleNamespace(x=float("nan"), y=2.0, z=3.0),
                           orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
    frame = SimpleNamespace(jpeg=b"abc", odom=SimpleNamespace(pose=pose))
    memory = simple_memory.SimpleMemory()
    with capture("web_input", base=archive) as web_run:
        try:
            with pytest.raises(simple_memory.SimpleMemoryUnavailable):
                record_frame(memory, frame)
        finally:
            memory.close()

    rows = [json.loads(line) for line in (web_run / "modules/spatial_memory.jsonl").read_text().splitlines()]
    row = next(row for row in rows if row.get("event") == "simple_memory_sample")
    assert row["outcome"] == "error"
    assert row["position_xyz"] == [None, 2.0, 3.0]


def test_memory_without_recording_skips_archive_initialization_and_diagnostic_events(
    worker_root, monkeypatch,
):
    def forbidden(*args, **kwargs):
        pytest.fail("the memory's archive path ran with recording disabled")

    monkeypatch.delenv("DIMOS_LOG_ROOT", raising=False)
    monkeypatch.setattr(simple_memory, "MemoryArchive", forbidden)
    monkeypatch.setattr(simple_memory, "event", forbidden)
    pose = SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0, z=3.0),
                           orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
    frame = SimpleNamespace(jpeg=b"abc", odom=SimpleNamespace(pose=pose))
    memory = simple_memory.SimpleMemory()
    try:
        assert record_frame(memory, frame)
        assert memory.query("box", lambda: 4.0) == [(1.0, 2.0, 0.25, 1.0)]
        assert not list(Path(memory._runtime.name).glob("*.log"))
    finally:
        memory.close()


def test_explicit_log_root_still_archives_memory_without_capture(worker_root, monkeypatch):
    archive = worker_root / "explicit-archive"
    monkeypatch.setenv("DIMOS_LOG_ROOT", str(archive))
    pose = SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0, z=3.0),
                           orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
    frame = SimpleNamespace(jpeg=b"abc", odom=SimpleNamespace(pose=pose))
    memory = simple_memory.SimpleMemory()
    try:
        assert record_frame(memory, frame)
        assert memory.query_hits("box", lambda: 4.0)[0].memory_id == "frame-test"
    finally:
        memory.close()

    images = list(archive.glob("spatial_memory/*/images/*.jpg"))
    assert [image.read_bytes() for image in images] == [frame.jpeg]
    worker_stderr = images[0].parent.parent.joinpath("stderr.log").read_text()
    assert "worker received record " in worker_stderr
    assert "worker received query " in worker_stderr


def test_sampling_accepts_translation_or_yaw_after_minimum_interval():
    from dimos.plugins.spatial_memory.simple_memory_worker import _sampling_rejection

    assert _sampling_rejection(None, None, None, 0.05, 15.0, 1.0) is None
    assert _sampling_rejection(0.05, 0.0, 1.0, 0.05, 15.0, 1.0) is None
    assert _sampling_rejection(0.0, 16.0, 1.0, 0.05, 15.0, 1.0) is None
    assert _sampling_rejection(0.049, 15.0, 1.0, 0.05, 15.0, 1.0) == "pose_change_below_minimum"
    assert _sampling_rejection(0.05, 16.0, 0.99, 0.05, 15.0, 1.0) == "interval_below_minimum"
