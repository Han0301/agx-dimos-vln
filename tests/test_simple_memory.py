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
    if request["action"] == "record":
        jpeg = base64.b64decode(request["jpeg"], validate=True)
        if pose is None:
            pose = request["pose"]
            send({"stored": True, "memory_id": frame_id})
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


def test_memory_records_observation_and_returns_valid_query_hits(worker_root):
    pose = SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0, z=3.0),
                           orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
    frame = SimpleNamespace(jpeg=b"abc", odom=SimpleNamespace(pose=pose))
    memory = simple_memory.SimpleMemory()
    try:
        assert memory.query("box", lambda: 4.0) == []
        assert memory.record(frame, lambda: 4.0)
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
            assert memory.record(frame, lambda: 4.0)
            assert not memory.record(frame, lambda: 4.0)
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
    image_path = archive / stored["image_path"]
    assert image_path.read_bytes() == jpeg
    assert image_path.parent.parent.joinpath("stderr.log").read_text().strip() == "memory worker log"
    assert len(list(image_path.parent.glob("*.jpg"))) == 1
    assert skipped["reason"] == "distance_below_minimum"
    lookup = next(row for row in rows if row.get("event") == "simple_memory_lookup")
    assert lookup["candidate_ids"] == ["frame-test"]


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
                memory.record(frame, lambda: 4.0)
        finally:
            memory.close()

    row = json.loads((web_run / "modules/spatial_memory.jsonl").read_text())
    assert row["event"] == "simple_memory_sample"
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
        assert memory.record(frame, lambda: 4.0)
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
        assert memory.record(frame, lambda: 4.0)
    finally:
        memory.close()

    images = list(archive.glob("spatial_memory/*/images/*.jpg"))
    assert [image.read_bytes() for image in images] == [frame.jpeg]
