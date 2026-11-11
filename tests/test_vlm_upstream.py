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
