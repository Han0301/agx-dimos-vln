import threading
from collections import deque
from types import SimpleNamespace

import pytest

from dimos.vln_task.observation import DEPTH_TOPIC, ODOM_TOPIC, RGB_TOPIC, Observations
from dimos.vln_task.search import exploration


def _sample(nanosec, frame_id):
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=1, nanosec=nanosec), frame_id=frame_id,
        ),
    )


def test_snapshot_pairs_post_settle_samples_even_with_a_closer_older_depth():
    from io import BytesIO
    from time import monotonic

    from PIL import Image

    observations = Observations.__new__(Observations)
    observations.settings = {"cache_s": 2, "gap_warning_ms": 150}
    observations.depth_scale = 1000
    observations.timeout = 0.1
    observations.cv = threading.Condition()
    observations.info = _sample(1, "camera")
    observations.info.width = observations.info.height = 2
    observations.info.k = [0] * 9
    observations.info.d = []
    observations.info.distortion_model = "plumb_bob"
    old_depth = _sample(20, "camera")
    new_depth = _sample(21, "camera")
    for depth, value in [(old_depth, 1000), (new_depth, 2000)]:
        with BytesIO() as data:
            Image.new("I;16", (2, 2), value).save(data, format="PNG")
            depth.data = data.getvalue()
    rgb = _sample(20, "camera")
    with BytesIO() as data:
        Image.new("RGB", (2, 2)).save(data, format="JPEG")
        rgb.data = data.getvalue()
    odom = _sample(20, "world")
    now = monotonic()
    observations.frames = {
        RGB_TOPIC: deque([(now, rgb)]),
        DEPTH_TOPIC: deque([(now - 0.2, old_depth), (now, new_depth)]),
        ODOM_TOPIC: deque([(now, odom)]),
    }

    result = observations.snapshot(
        lambda: 1, after_stamp_ns=1_000_000_010, received_after=now - 0.1, max_age_s=0.5,
    )

    assert result.stamp_ns == 1_000_000_020
    assert result.depth[0, 0] == 2
    assert result.received_at >= now - 0.1


def test_stopped_capture_fences_at_stop_and_does_not_pre_wait(monkeypatch):
    monkeypatch.setattr(exploration.time, "monotonic", lambda: 40.0)
    calls = []
    observed = object()

    def newest_stamp():
        calls.append("stop stamp")
        return 110

    def observe(remaining, **options):
        calls.append("observe")
        assert options == {
            "after_stamp_ns": 110,
            "received_after": pytest.approx(40.15),
            "max_age_s": exploration.TRACKING_MAX_AGE_S,
        }
        return observed

    result = exploration.observe_stopped(observe, newest_stamp, 100, lambda: 1, 0.15)

    assert result is observed
    assert calls == ["stop stamp", "observe"]
