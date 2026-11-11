import json
from collections import deque
from io import BytesIO
from threading import Condition
from types import SimpleNamespace as NS

import numpy as np
import pytest
from PIL import Image


def observe_events(monkeypatch, module):
    events = []
    monkeypatch.setattr(module, "event", lambda _, name, **fields: events.append((name, fields)),
                        raising=False)
    monkeypatch.setattr(module, "recording_enabled", lambda: True, raising=False)
    return events


def test_detection_logs_original_and_final_verdicts_with_depth_statistics(monkeypatch):
    from dimos.plugins.vln_task import preview

    events = observe_events(monkeypatch, preview)
    frame = NS(
        jpeg=b"frame", stamp_ns=100, wait_ms=1, depth_gap_ms=2, odom_gap_ms=3,
        depth=np.full((100, 100), 2.), k=(100, 0, 50, 0, 100, 50, 0, 0, 1),
        distortion=(), distortion_model="",
        odom=NS(header=NS(frame_id="world"), pose=NS(
            position=NS(x=1., y=2., z=0.),
            orientation=NS(x=0., y=0., z=0., w=1.),
        )),
    )
    vlm = NS(config={"timeout_s": 1}, query=lambda *a, **kw: {
        "text": json.dumps([{"bbox_2d": [400, 400, 600, 600],
                            "label": "cabinet", "appearance_match": "uncertain"}]),
        "elapsed_ms": 5,
    })
    skill = preview.NavigationPreview(
        vlm, lambda _: frame, None, None, {"translation": [0, 0, 0], "pitch": 0}, 0.6,
        verify=lambda *a, **kw: ("match", 4),
    )

    result = skill.detect(frame, "fire extinguisher cabinet", lambda: 2)

    assert result["status"] == "localized"
    detection = dict(events)["detection_result"]
    assert detection["detector_verdict"] == "uncertain"
    assert detection["final_verdict"] == "match"
    assert detection["roi_valid_fraction"] == 1.
    assert detection["median_depth_m"] == 2.
    assert detection["world_frame"] == "world"
    assert detection["target_xyz"] == pytest.approx([3., 2., 0.])
    assert detection["goal_xy"] == pytest.approx([2.4, 2.])
    assert "roi_valid_fraction" not in result


def test_observation_logs_exact_selected_sample_stamps_not_latest_depth(monkeypatch):
    from dimos.plugins.vln_task import observation as module

    events = observe_events(monkeypatch, module)
    monkeypatch.setattr(module.time, "monotonic", lambda: 10.)
    observations = module.Observations.__new__(module.Observations)
    observations.cv = Condition()
    observations.settings = {"cache_s": 3., "gap_warning_ms": 150}
    observations.timeout = 2.
    observations.depth_scale = 1000.
    observations.info_received = 9.7

    def header(ns, frame_id="camera"):
        return NS(stamp=NS(sec=1, nanosec=ns), frame_id=frame_id)

    def image(fmt, **kwargs):
        output = BytesIO()
        Image.new("I;16" if fmt == "PNG" else "RGB", (2, 2)).save(output, fmt, **kwargs)
        return output.getvalue()

    observations.info = NS(header=header(1), width=2, height=2, k=(),
                           distortion_model="", d=())
    observations.frames = {
        module.RGB_TOPIC: deque([(9.8, NS(header=header(100), data=image("JPEG")))]),
        module.DEPTH_TOPIC: deque([
            (9.81, NS(header=header(120), data=image("PNG"))),
            (9.99, NS(header=header(500), data=image("PNG"))),
        ]),
        module.ODOM_TOPIC: deque([(9.79, NS(header=header(80, "world")))]),
    }

    assert observations.snapshot(lambda: 2).stamp_ns == 1_000_000_100

    row = dict(events)["observation_selected"]
    assert row["rgb_stamp_ns"] == 1_000_000_100
    assert row["depth_stamp_ns"] == 1_000_000_120
    assert row["odom_stamp_ns"] == 1_000_000_080
    assert row["camera_info_stamp_ns"] == 1_000_000_001
    assert row["depth_age_ms"] == pytest.approx(190.)
    assert row["depth_offset_ms"] > 0
    assert row["odom_offset_ms"] < 0
