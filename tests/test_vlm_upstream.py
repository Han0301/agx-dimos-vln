import base64
import json

import cv2
import numpy as np
import pytest
import websocket

URL = "wss://fixture.invalid/api-ws/v1/realtime?model=test-model"
KEY = "test-key"
EXPECTED_TIMINGS = {
    "queue",
    "upstream_connect",
    "session_configure",
    "frame_pacing",
    "image_commit",
    "first_response",
    "complete_output",
    "teardown",
}


class SocketFixture:
    def __init__(self, events):
        self.events = iter(events)
        self.sent = []
        self.timeouts = []
        self.closed = False

    def settimeout(self, timeout):
        self.timeouts.append(timeout)

    def recv(self):
        event = next(self.events)
        if isinstance(event, Exception):
            raise event
        return json.dumps(event)

    def send(self, message):
        self.sent.append(json.loads(message))

    def close(self, *, timeout):
        self.closed = True


def successful_exchange():
    return [
        {"type": "session.created"},
        {"type": "session.updated"},
        {"type": "input_audio_buffer.committed"},
        {"type": "response.text.delta", "delta": "white"},
        {"type": "response.text.delta", "delta": " bin"},
        {
            "type": "response.done",
            "response": {"status": "completed", "usage": {"output_tokens": 2}},
        },
    ]


def client_with(monkeypatch, *sockets, timeout_s=2.0):
    from dimos.plugins.vlm.upstream import OmniClient

    remaining = iter(sockets)

    def connect(url, *, header, timeout):
        assert url == URL
        assert header == ["Authorization: Bearer test-key"]
        assert 0 < timeout <= timeout_s
        return next(remaining)

    monkeypatch.setattr(websocket, "create_connection", connect)
    return OmniClient("test-model", URL, KEY, timeout_s)


def test_realtime_image_requests_isolate_sessions_and_preserve_protocol(monkeypatch):
    first = SocketFixture(successful_exchange())
    second = SocketFixture(successful_exchange())
    client = client_with(monkeypatch, first, second)
    timings = {}
    image = np.zeros((480, 848, 3), dtype=np.uint8)

    result = client.infer(image, "find white bin", timing=timings)
    client.infer(image, "verify bin", timing={})

    assert result["response"] == "white bin"
    assert result["model"] == "test-model"
    assert result["usage"] == {"output_tokens": 2}
    assert 0 <= result["first_text_ms"] <= result["latency_ms"]
    assert set(timings) == EXPECTED_TIMINGS
    assert all(value >= 0 for value in timings.values())
    assert first.closed and second.closed and not client.connected
    assert [message["type"] for message in first.sent] == [
        "session.update",
        "input_audio_buffer.append",
        "input_image_buffer.append",
        "input_audio_buffer.commit",
        "response.create",
    ]
    assert first.sent[0]["session"] == {
        "modalities": ["text"],
        "turn_detection": None,
        "input_audio_format": "pcm",
        "instructions": "find white bin",
    }
    assert second.sent[0]["session"]["instructions"] == "verify bin"
    assert base64.b64decode(first.sent[1]["audio"]) == bytes(6400)
    sent_image = cv2.imdecode(
        np.frombuffer(base64.b64decode(first.sent[2]["image"]), np.uint8),
        cv2.IMREAD_COLOR,
    )
    assert sent_image.shape == (480, 848, 3)


def test_upstream_errors_are_sanitized_and_release_the_session(monkeypatch):
    failed = SocketFixture(
        [{"type": "error", "error": {"message": "secret test-key"}}]
    )
    healthy = SocketFixture(successful_exchange())
    client = client_with(monkeypatch, failed, healthy)
    image = np.zeros((10, 10, 3), dtype=np.uint8)

    with pytest.raises(RuntimeError) as error:
        client.infer(image, "first", timing={})

    assert KEY not in str(error.value)
    assert failed.closed and not client.connected
    assert client.infer(image, "second", timing={})["response"] == "white bin"


def test_request_timeout_covers_all_received_events(monkeypatch):
    from dimos.plugins.vlm import upstream

    class Clock:
        now = 10.0

        def monotonic(self):
            return self.now

    clock = Clock()

    class SlowSocket(SocketFixture):
        def recv(self):
            clock.now += 0.8
            return super().recv()

    socket = SlowSocket(successful_exchange())
    client = client_with(monkeypatch, socket, timeout_s=1.0)
    monkeypatch.setattr(upstream, "time", clock)
    image = np.zeros((10, 10, 3), dtype=np.uint8)

    with pytest.raises(TimeoutError):
        client.infer(image, "first", timing={})

    assert any(timeout == pytest.approx(0.2) for timeout in socket.timeouts)
    assert socket.closed and not client.connected


def test_vlm_logs_exact_realtime_websocket_input(monkeypatch, tmp_path):
    from dimos.plugins.runtime_log import capture, event_context
    from dimos.plugins.vlm import credentials, upstream
    from dimos.plugins.vlm.client import VLMClient

    socket = SocketFixture(successful_exchange())
    client = client_with(monkeypatch, socket)
    monkeypatch.setattr(upstream, "OmniClient", lambda **_: client)
    monkeypatch.setattr(credentials, "load_upstream", lambda _: {})
    image = np.zeros((48, 64, 3), dtype=np.uint8)
    image[:, :32] = [12, 77, 193]
    ok, source = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 97])
    assert ok

    with capture("web_input", base=tmp_path) as run:
        with event_context(observation_id="obs-a"):
            with VLMClient({"backend": "dashscope", "credentials_file": "fixture", "timeout_s": 2.0}) as vlm:
                assert vlm.query(source.tobytes(), "find white bin", source="d455")["text"] == "white bin"

    image_message = next(message for message in socket.sent if message["type"] == "input_image_buffer.append")
    sent = base64.b64decode(image_message["image"])
    assert sent != source.tobytes()
    records = [json.loads(line) for line in (run / "modules/vlm.jsonl").read_text().splitlines()]
    call = next(row for row in records if row.get("event") == "vlm_call_finished")
    assert call["observation_id"] == "obs-a"
    assert call["input_image_archive_status"] == "queued"
    assert (run / call["input_image_artifact"]).read_bytes() == sent


def test_realtime_without_recorder_does_not_decode_for_archiving(monkeypatch):
    from dimos.plugins import runtime_log
    from dimos.plugins.vlm import client as vlm_client, credentials, upstream

    monkeypatch.setattr(runtime_log, "_active_components", None)
    socket = SocketFixture(successful_exchange())
    backend = client_with(monkeypatch, socket)
    monkeypatch.setattr(upstream, "OmniClient", lambda **_: backend)
    monkeypatch.setattr(credentials, "load_upstream", lambda _: {})
    monkeypatch.setattr(vlm_client, "diagnostic_artifact", lambda *a, **kw: pytest.fail("archive attempted"))
    monkeypatch.setattr(
        upstream, "base64",
        type("EncodeOnly", (), {
            "b64encode": staticmethod(base64.b64encode),
            "b64decode": staticmethod(lambda *_: pytest.fail("log-only Base64 decode")),
        }),
    )
    ok, image = cv2.imencode(".jpg", np.zeros((16, 16, 3), dtype=np.uint8))
    assert ok
    with vlm_client.VLMClient({"backend": "dashscope", "credentials_file": "fixture", "timeout_s": 2.0}) as vlm:
        assert vlm.query(image.tobytes(), "find white bin")["text"] == "white bin"
    assert any(message["type"] == "input_image_buffer.append" for message in socket.sent)


def test_dual_realtime_images_share_one_turn_and_archive_in_send_order(monkeypatch, tmp_path):
    from dimos.plugins.runtime_log import capture
    from dimos.plugins.vlm import credentials, upstream
    from dimos.plugins.vlm.client import VLMClient

    class Clock:
        now = 10.0

        def monotonic(self):
            return self.now

        def sleep(self, seconds):
            self.now += seconds

    clock = Clock()
    socket = SocketFixture(successful_exchange())
    backend = client_with(monkeypatch, socket)
    monkeypatch.setattr(upstream, "time", clock)
    monkeypatch.setattr(upstream, "OmniClient", lambda **_: backend)
    monkeypatch.setattr(credentials, "load_upstream", lambda _: {})
    reference = cv2.imencode(".png", np.full((24, 32, 3), 47, np.uint8))[1].tobytes()
    scene = cv2.imencode(".jpg", np.full((48, 64, 3), 201, np.uint8))[1].tobytes()

    with capture("web_input", base=tmp_path) as run:
        with VLMClient({"backend": "dashscope", "credentials_file": "fixture", "timeout_s": 2.0}) as vlm:
            assert vlm.supports_multi_image
            result = vlm.query_images((reference, scene), "compare the two images", stamp_ns=91)

    assert result["text"] == "white bin"
    assert result["image_sizes"] == [[32, 24], [64, 48]]
    assert result["stamp_ns"] == 91
    assert [message["type"] for message in socket.sent] == [
        "session.update",
        "input_audio_buffer.append",
        "input_image_buffer.append",
        "input_audio_buffer.append",
        "input_image_buffer.append",
        "input_audio_buffer.commit",
        "response.create",
    ]
    assert socket.sent[0]["session"]["instructions"] == "compare the two images"
    assert clock.now == pytest.approx(11.0)
    assert len(base64.b64decode(socket.sent[3]["audio"])) == 32_000
    sent = [base64.b64decode(socket.sent[index]["image"]) for index in (2, 4)]
    frames = [cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR) for jpeg in sent]
    assert [frame.shape for frame in frames] == [(24, 32, 3), (48, 64, 3)]
    assert [int(frame.mean()) for frame in frames] == [47, 201]
    records = [json.loads(line) for line in (run / "modules/vlm.jsonl").read_text().splitlines()]
    call = next(row for row in records if row.get("event") == "vlm_call_finished")
    assert [(run / entry["artifact"]).read_bytes() for entry in call["input_images"]] == sent
    assert (run / call["input_image_artifact"]).read_bytes() == sent[-1]
    assert socket.closed and not backend.connected


def test_dual_realtime_deadline_prevents_committing_incomplete_turn(monkeypatch):
    socket = SocketFixture(successful_exchange())
    client = client_with(monkeypatch, socket, timeout_s=0.5)
    frame = np.zeros((16, 16, 3), dtype=np.uint8)

    with pytest.raises(TimeoutError):
        client.infer_images((frame, frame), "compare", timing={})

    assert [message["type"] for message in socket.sent] == [
        "session.update",
        "input_audio_buffer.append",
        "input_image_buffer.append",
    ]
    assert socket.closed and not client.connected


def test_dual_realtime_bounds_dimensions_and_size_of_each_image(monkeypatch):
    from dimos.plugins.vlm import upstream

    socket = SocketFixture(successful_exchange())
    client = client_with(monkeypatch, socket)
    monkeypatch.setattr(upstream, "_FRAME_INTERVAL_S", 0)
    rng = np.random.default_rng(42)
    images = (
        rng.integers(0, 256, size=(1200, 2400, 3), dtype=np.uint8),
        rng.integers(0, 256, size=(1080, 1920, 3), dtype=np.uint8),
    )

    client.infer_images(images, "compare", timing={})

    sent = [
        base64.b64decode(message["image"])
        for message in socket.sent
        if message["type"] == "input_image_buffer.append"
    ]
    assert len(sent) == 2
    for image, jpeg in zip(images, sent):
        assert len(jpeg) <= 190_000
        decoded = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        height, width = decoded.shape[:2]
        assert height <= 1080 and width <= 1920
        assert width / height == pytest.approx(image.shape[1] / image.shape[0], abs=0.004)
