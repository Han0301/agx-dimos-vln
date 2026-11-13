"""Omni Realtime image input with text output and one session per request."""

import base64
import json
import logging
import threading
import time

import cv2
import websocket

_SILENT_AUDIO = base64.b64encode(bytes(6400)).decode("ascii")
_FRAME_INTERVAL_S = 1.0
# One second of silence in the 16 kHz, 16-bit PCM input timeline.
_FRAME_GAP_AUDIO = base64.b64encode(bytes(32_000)).decode("ascii")


def encode_image(image, *, fit_frame=False):
    height, width = image.shape[:2]
    short, long = sorted((height, width))
    fit_scale = min(1.0, 1080 / short, 1920 / long) if fit_frame else 1.0
    ratios = (1.0, 0.75, 0.56) if fit_frame else (1.0,)
    for ratio in ratios:
        scale = fit_scale * ratio
        frame = image if scale == 1 else cv2.resize(
            image, (max(1, round(width * scale)), max(1, round(height * scale))),
            interpolation=cv2.INTER_AREA,
        )
        for quality in (80, 65, 45, 25):
            ok, data = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
            if ok and len(data) <= 190_000:
                return base64.b64encode(data).decode("ascii")
    raise RuntimeError("JPEG exceeds the Realtime image limit")


def _remaining(deadline):
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise TimeoutError("VLM upstream request deadline exceeded")
    return remaining


def _send(connection, deadline, kind, **payload):
    connection.settimeout(_remaining(deadline))
    connection.send(json.dumps({"type": kind, **payload}))


def _receive(connection, deadline):
    connection.settimeout(_remaining(deadline))
    event = json.loads(connection.recv())
    _remaining(deadline)
    if not isinstance(event, dict) or not isinstance(event.get("type"), str):
        raise RuntimeError("Invalid VLM upstream event")
    if event["type"] == "error":
        raise RuntimeError("VLM upstream rejected the request")
    return event


def _wait_for(connection, deadline, kind):
    while _receive(connection, deadline)["type"] != kind:
        pass


logger = logging.getLogger(__name__)


class VLMConnectionError(RuntimeError):
    """A transient failure before sending the inference input."""


class OmniClient:
    def __init__(self, model, websocket_url, api_key, timeout_s):
        self.model = model
        self._url = websocket_url
        self._api_key = api_key
        self._timeout = timeout_s
        self._lock = threading.Lock()
        self._last_frame = None
        self._connected = False

    @property
    def connected(self):
        return self._connected

    def infer(self, image, prompt, *, timing, timeout=None, archive_input=None):
        return self.infer_images(
            (image,), prompt, timing=timing, timeout=timeout, archive_input=archive_input,
        )

    def infer_images(self, images, prompt, *, timing, timeout=None, archive_input=None):
        if len(images) not in (1, 2):
            raise ValueError("Realtime inference expects one or two ordered images")
        queued = time.monotonic()
        budget = self._timeout if timeout is None else min(self._timeout, timeout)
        deadline = queued + budget
        if not self._lock.acquire(timeout=_remaining(deadline)):
            raise TimeoutError("VLM upstream request timed out in the request queue")

        start = time.monotonic()
        timing["queue"] = start - queued
        connection = None
        try:
            for attempt in (1, 2):
                phase = "connect"
                connection_deadline = min(deadline, time.monotonic() + 5.0) if attempt == 1 else deadline
                try:
                    connection = websocket.create_connection(
                        self._url,
                        header=["Authorization: Bearer " + self._api_key],
                        timeout=_remaining(connection_deadline),
                    )
                    phase = "session_created"
                    _wait_for(connection, connection_deadline, "session.created")
                    break
                except (websocket.WebSocketTimeoutException, websocket.WebSocketConnectionClosedException,
                        ConnectionError, TimeoutError) as exc:
                    logger.warning("VLM connection failed phase=%s attempt=%d error_type=%s",
                                   phase, attempt, type(exc).__name__)
                    if attempt == 2 or deadline - time.monotonic() <= 0.2:
                        raise VLMConnectionError("VLM upstream connection unavailable") from None
                    if connection is not None:
                        connection.shutdown()
                        connection = None
                    time.sleep(0.2)
            self._connected = True
            connected = time.monotonic()
            timing["upstream_connect"] = connected - start

            _send(
                connection,
                deadline,
                "session.update",
                session={
                    "modalities": ["text"],
                    "turn_detection": None,
                    "input_audio_format": "pcm",
                    "instructions": prompt,
                },
            )
            _wait_for(connection, deadline, "session.updated")
            configured = time.monotonic()
            timing["session_configure"] = configured - connected

            timing["frame_pacing"] = 0.0
            for index, image in enumerate(images):
                # Respect the documented 1 fps pacing and place each image at
                # a distinct position on the audio timeline in the same turn.
                before_pacing = time.monotonic()
                if self._last_frame is not None:
                    pause = _FRAME_INTERVAL_S - (before_pacing - self._last_frame)
                    if pause > 0:
                        if pause >= _remaining(deadline):
                            raise TimeoutError("VLM upstream request timed out before sending an image")
                        time.sleep(pause)
                timing["frame_pacing"] += time.monotonic() - before_pacing
                audio = _SILENT_AUDIO if index == 0 else _FRAME_GAP_AUDIO
                _send(connection, deadline, "input_audio_buffer.append", audio=audio)
                encoded_image = encode_image(image, fit_frame=True)
                if archive_input is not None:
                    archive_input(base64.b64decode(encoded_image))
                _send(connection, deadline, "input_image_buffer.append", image=encoded_image)
                self._last_frame = time.monotonic()
            _send(connection, deadline, "input_audio_buffer.commit")
            _wait_for(connection, deadline, "input_audio_buffer.committed")
            requested = time.monotonic()
            timing["image_commit"] = requested - configured - timing["frame_pacing"]
            _send(connection, deadline, "response.create")

            chunks = []
            first = None
            while True:
                event = _receive(connection, deadline)
                if event["type"] == "response.text.delta":
                    delta = event.get("delta")
                    if not isinstance(delta, str):
                        raise RuntimeError("Invalid VLM upstream text delta")
                    if first is None:
                        first = time.monotonic()
                        timing["first_response"] = first - requested
                    chunks.append(delta)
                elif event["type"] == "response.done":
                    response = event.get("response")
                    if not isinstance(response, dict) or response.get("status") != "completed":
                        raise RuntimeError("VLM upstream response did not complete")
                    break

            text = "".join(chunks)
            if first is None or not text.strip():
                raise RuntimeError("VLM upstream returned empty text")
            completed = time.monotonic()
            timing["complete_output"] = completed - first
            return {
                "response": text,
                "model": self.model,
                "latency_ms": round((completed - start) * 1000, 1),
                "first_text_ms": round((first - start) * 1000, 1),
                "usage": response.get("usage", {}),
                "response_id": response.get("id"),
            }
        except websocket.WebSocketTimeoutException:
            raise TimeoutError("VLM upstream request deadline exceeded") from None
        except (websocket.WebSocketException, OSError, ValueError, cv2.error) as exc:
            if isinstance(exc, TimeoutError):
                raise TimeoutError("VLM upstream request deadline exceeded") from None
            raise RuntimeError("VLM upstream transport or image encoding failed") from None
        finally:
            teardown = time.monotonic()
            try:
                if connection is not None:
                    # The provider retains visual and system state across turns.
                    timeout = min(1.0, max(0.0, deadline - teardown))
                    connection.settimeout(timeout)
                    connection.close(timeout=timeout)
            except (websocket.WebSocketException, OSError):
                pass
            finally:
                self._connected = False
                timing["teardown"] = time.monotonic() - teardown
                self._lock.release()
