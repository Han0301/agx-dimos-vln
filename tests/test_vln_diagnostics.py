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


def test_detection_logs_bbox_and_raw_reply_with_depth_statistics(monkeypatch):
    from dimos.vln_task import perception, preview

    events = observe_events(monkeypatch, perception)
    attachments = observe_artifacts(monkeypatch, perception)
    frame = NS(
        jpeg=b"frame", stamp_ns=100, wait_ms=1, depth_gap_ms=2, odom_gap_ms=3,
        depth=np.full((100, 100), 2.), k=(100, 0, 50, 0, 100, 50, 0, 0, 1),
        distortion=(), distortion_model="",
        odom=NS(header=NS(frame_id="world"), pose=NS(
            position=NS(x=1., y=2., z=0.),
            orientation=NS(x=0., y=0., z=0., w=1.),
        )),
    )
    response = json.dumps({"name": "cabinet", "bbox": [400, 400, 600, 600]})
    vlm = NS(config={"timeout_s": 1}, query=lambda *a, **kw: {
        "text": response, "elapsed_ms": 5,
    })
    skill = preview.NavigationPreview(
        vlm, lambda _: frame, None, None, {"translation": [0, 0, 0], "pitch": 0}, 0.6,
    )

    result = skill.detect(frame, "fire extinguisher cabinet", lambda: 2)

    assert result["status"] == "localized"
    detection = dict(events)["detection_result"]
    assert detection["bbox_2d"] == result["bbox_2d"] == [400, 400, 600, 600]
    assert "detector_verdict" not in detection
    assert "final_verdict" not in detection
    assert detection["roi_valid_fraction"] == 1.
    assert detection["median_depth_m"] == 2.
    assert detection["world_frame"] == "world"
    assert detection["target_xyz"] == pytest.approx([3., 2., 0.])
    assert detection["goal_xy"] == pytest.approx([2.4, 2.])
    assert "roi_valid_fraction" not in result
    assert detection["camera_extrinsics"] == {"translation": [0, 0, 0], "pitch": 0}
    reply = next(entry for entry in attachments if entry[0] == detection["vlm_response_artifact"])
    assert reply[1] == response


@pytest.mark.parametrize("recording", [False, True])
def test_motion_observations_require_samples_without_a_receive_age_limit(monkeypatch, recording):
    from dimos.vln_task import observation as module

    observe_events(monkeypatch, module)
    monkeypatch.setattr(module, "recording_enabled", lambda: recording)
    monkeypatch.setattr(module.time, "monotonic", lambda: 10.)
    observations = module.Observations(
        NS(create_subscription=lambda *args: None), {}, 1000., 1.,
    )
    sample = NS(header=NS(stamp=NS(sec=1, nanosec=0)))
    for topic in (module.RGB_TOPIC, module.ODOM_TOPIC, module.INFO_TOPIC):
        observations._receiver(topic)(sample)

    assert not observations.motion_inputs_ready()

    observations._receiver(module.DEPTH_TOPIC)(sample)
    monkeypatch.setattr(module.time, "monotonic", lambda: 12.)
    assert observations.motion_inputs_ready()


def test_observation_logs_exact_selected_sample_stamps_not_latest_depth(monkeypatch):
    from dimos.vln_task import observation as module

    events = observe_events(monkeypatch, module)
    attachments = observe_artifacts(monkeypatch, module)
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

    observations.info = NS(
        header=header(1), width=2, height=2, k=(10, 0, 1, 0, 10, 1, 0, 0, 1),
        distortion_model="plumb_bob", d=(0.1, 0.2, 0.3, 0.4, 0.5),
        r=(1, 0, 0, 0, 1, 0, 0, 0, 1), p=(10, 0, 1, 0, 0, 10, 1, 0, 0, 0, 1, 0),
    )
    rgb = image("JPEG")
    matched_depth = bytes(12) + image("PNG")
    odom = NS(
        header=header(80, "world"), pose=NS(
            position=NS(x=1, y=2, z=3), orientation=NS(x=0, y=0, z=0, w=1),
        ),
    )
    observations.frames = {
        module.RGB_TOPIC: deque([(9.8, NS(header=header(100), data=rgb, format="jpeg"))]),
        module.DEPTH_TOPIC: deque([
            (9.81, NS(header=header(120), data=matched_depth, format="16UC1; compressedDepth png")),
            (9.99, NS(header=header(500), data=image("PNG"))),
        ]),
        module.ODOM_TOPIC: deque([(9.79, odom)]),
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
    by_path = {path: (data, metadata) for path, data, metadata in attachments}
    assert by_path[row["rgb_diagnostic_artifact"]][0] == rgb
    assert by_path[row["depth_diagnostic_artifact"]][0] == matched_depth
    assert by_path[row["depth_diagnostic_artifact"]][1]["source_stamp_ns"] == row["depth_stamp_ns"]
    assert row["camera_info"]["k"] == [10, 0, 1, 0, 10, 1, 0, 0, 1]
    assert row["camera_info"]["d"] == [0.1, 0.2, 0.3, 0.4, 0.5]
    assert row["camera_info"]["p"] == [10, 0, 1, 0, 0, 10, 1, 0, 0, 0, 1, 0]
    assert row["odom_pose"] == {"position_xyz": [1, 2, 3], "orientation_xyzw": [0, 0, 0, 1]}
    assert row["depth_scale"] == 1000.
    assert row["depth_format"] == "16UC1; compressedDepth png"


def observe_artifacts(monkeypatch, module):
    attachments = []

    def diagnostic_artifact(filename, data, **metadata):
        path = f"diagnostics/{filename}"
        attachments.append((path, data, metadata))
        return path

    monkeypatch.setattr(module, "diagnostic_artifact", diagnostic_artifact, raising=False)
    return attachments


def test_detection_logs_reply_for_no_candidate(monkeypatch):
    from dimos.vln_task import perception

    events = observe_events(monkeypatch, perception)
    attachments = observe_artifacts(monkeypatch, perception)
    frame = NS(jpeg=b"frame", stamp_ns=100, wait_ms=1, depth_gap_ms=2, odom_gap_ms=3)
    vlm = NS(config={"timeout_s": 1}, query=lambda *a, **kw: {"text": "None", "elapsed_ms": 5})
    detector = perception.ObjectPerception(vlm, {"translation": [0, 0, 0], "pitch": 0}, 0.6)

    assert detector.detect(frame, "cabinet", lambda: 2)["status"] == "not_visible"
    detection = dict(events)["detection_result"]
    assert detection["status"] == "not_visible"
    assert detection["bbox_2d"] is None
    assert [(path, data) for path, data, _ in attachments] == [(detection["vlm_response_artifact"], "None")]


def test_successful_verification_logs_exact_vlm_crop_and_reply(monkeypatch):
    from dimos.vln_task.search import verification

    events = observe_events(monkeypatch, verification)
    attachments = observe_artifacts(monkeypatch, verification)
    output = BytesIO()
    Image.new("RGB", (100, 100)).save(output, "JPEG")
    frame = NS(jpeg=output.getvalue(), stamp_ns=100)
    requests = []

    def query(image, *_args, **_kwargs):
        requests.append(image)
        return {"text": '{"match":"yes"}', "elapsed_ms": 5}

    vlm = NS(config={"timeout_s": 1}, query=query)
    assert verification.verify_candidate(vlm, frame, "cabinet", [100, 100, 700, 700], lambda: 2) == ("match", 5)
    result = dict(events)["visual_verification_result"]
    by_path = {path: (data, metadata) for path, data, metadata in attachments}
    assert by_path[result["crop_diagnostic_artifact"]][0] == requests[0]
    assert by_path[result["vlm_response_artifact"]][0] == '{"match":"yes"}'
    assert by_path[result["crop_diagnostic_artifact"]][1]["verification_id"] == result["verification_id"]


def test_snapshot_without_recording_reads_each_image_once(monkeypatch):
    from dimos.vln_task import observation as module

    def forbidden(*args, **kwargs):
        pytest.fail("the observation's archive path ran with recording disabled")

    class CameraInfo(NS):
        @property
        def r(self):
            pytest.fail("recording-only rectification metadata was read")

    class CountedImage:
        copies = 0

        def __init__(self, format):
            output = BytesIO()
            Image.new("I;16" if format == "PNG" else "RGB", (2, 2)).save(output, format)
            self.data = output.getvalue()

        def __bytes__(self):
            self.copies += 1
            return self.data

    monkeypatch.setattr(module, "recording_enabled", lambda: False)
    monkeypatch.setattr(module, "diagnostic_artifact", forbidden)
    monkeypatch.setattr(module.time, "monotonic", lambda: 10.)
    rgb, depth = CountedImage("JPEG"), CountedImage("PNG")
    camera_header = NS(stamp=NS(sec=1, nanosec=100), frame_id="camera")
    world_header = NS(stamp=NS(sec=1, nanosec=100), frame_id="world")
    observations = module.Observations.__new__(module.Observations)
    observations.cv = Condition()
    observations.settings = {"cache_s": 3., "gap_warning_ms": 150}
    observations.timeout = 2.
    observations.depth_scale = 1000.
    observations.info = CameraInfo(
        header=camera_header, width=2, height=2, k=(10, 0, 1, 0, 10, 1, 0, 0, 1),
        distortion_model="plumb_bob", d=(),
    )
    observations.frames = {
        module.RGB_TOPIC: deque([(9.8, NS(header=camera_header, data=rgb))]),
        module.DEPTH_TOPIC: deque([(9.8, NS(header=camera_header, data=depth))]),
        module.ODOM_TOPIC: deque([(9.8, NS(header=world_header))]),
    }

    sample = observations.snapshot(lambda: 2)
    assert sample.jpeg == rgb.data and sample.depth.shape == (2, 2)
    assert (rgb.copies, depth.copies) == (1, 1)


def test_verification_without_recording_avoids_diagnostic_context(monkeypatch):
    from dimos.vln_task.search import verification

    def forbidden(*args, **kwargs):
        pytest.fail("the verification's log context ran with recording disabled")

    monkeypatch.setattr(verification, "recording_enabled", lambda: False)
    monkeypatch.setattr(verification, "event_context", forbidden)
    monkeypatch.setattr(verification, "diagnostic_artifact", forbidden)
    output = BytesIO()
    Image.new("RGB", (100, 100)).save(output, "JPEG")
    frame = NS(jpeg=output.getvalue(), stamp_ns=100)

    def query(image, *_args, **_kwargs):
        assert image.startswith(b"\xff\xd8")
        return {"text": '{"match":"yes"}', "elapsed_ms": 5}

    vlm = NS(config={"timeout_s": 1}, query=query)
    assert verification.verify_candidate(vlm, frame, "cabinet", [100, 100, 700, 700], lambda: 2) == ("match", 5)


def test_detection_without_recording_avoids_diagnostic_artifacts(monkeypatch):
    from dimos.vln_task import perception

    def forbidden(*args, **kwargs):
        pytest.fail("the detection attempted to archive diagnostics with recording disabled")

    monkeypatch.setattr(perception, "recording_enabled", lambda: False)
    monkeypatch.setattr(perception, "diagnostic_artifact", forbidden)
    monkeypatch.setattr(perception, "artifact", forbidden)
    frame = NS(jpeg=b"frame", stamp_ns=100, wait_ms=1, depth_gap_ms=2, odom_gap_ms=3)
    vlm = NS(config={"timeout_s": 1}, query=lambda *a, **kw: {"text": "None", "elapsed_ms": 5})

    detector = perception.ObjectPerception(vlm, {"translation": [0, 0, 0], "pitch": 0}, 0.6)
    assert detector.detect(frame, "cabinet", lambda: 2)["status"] == "not_visible"
