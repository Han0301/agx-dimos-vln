"""Explicit-image VLM using DashScope directly or the compatible HTTP protocol."""

import base64
import io
import logging
import time
from pathlib import Path
from uuid import uuid4

from httpx import HTTPError, HTTPStatusError

from dimos.plugins._agent_common import HTTPModel, validate_image
from dimos.plugins.runtime_log import event, recording_enabled

logger = logging.getLogger(__name__)


class VLMClient(HTTPModel):
    def __init__(self, config, http=None):
        self.upstream = None
        if config.get("backend", "http") == "http":
            super().__init__(config, http)
        else:
            if http is not None:
                raise ValueError("An HTTP transport requires the VLM HTTP backend")
            from .credentials import load_upstream
            from .upstream import OmniClient

            self.config = config
            self.http = None
            self.owns_http = False
            self.upstream = OmniClient(
                **load_upstream(Path(config["credentials_file"]).expanduser()),
                timeout_s=config["timeout_s"],
            )

    def query(
        self, image, prompt, *, source="explicit_image", stamp_ns=None, timeout=None
    ):
        if not recording_enabled():
            return self._query(image, prompt, source, stamp_ns, timeout, None)
        started = time.perf_counter()
        stages = {}
        fields = {
            "call_id": uuid4().hex, "source": source, "rgb_stamp_ns": stamp_ns,
            "backend": self.config.get("backend", "http"), "timeout_s": timeout,
            "image_bytes": len(image) if isinstance(image, bytes) else None,
        }
        try:
            result = self._query(image, prompt, source, stamp_ns, timeout, (fields, stages))
        except (HTTPError, ValueError, RuntimeError, TimeoutError) as exc:
            event(logger, "vlm_call_finished", status="failed",
                  error_type=type(exc).__name__,
                  http_status=exc.response.status_code if isinstance(exc, HTTPStatusError) else None,
                  elapsed_ms=(time.perf_counter() - started) * 1000,
                  stages_ms={stage: value * 1000 for stage, value in stages.items()}, **fields)
            raise
        event(logger, "vlm_call_finished", status="succeeded", elapsed_ms=result["elapsed_ms"],
              stages_ms={stage: value * 1000 for stage, value in stages.items()}, **fields)
        return result

    def _query(self, image, prompt, source, stamp_ns, timeout, diagnostics):
        started = time.perf_counter()
        size = validate_image(image)
        if diagnostics is not None:
            fields, stages = diagnostics
            fields["image_size"] = size
        if not isinstance(prompt, str) or not prompt.strip():
            raise ValueError("VLM prompt must be nonempty")
        if stamp_ns is not None and (type(stamp_ns) is not int or stamp_ns < 0):
            raise ValueError("stamp_ns must be a nonnegative integer")
        # Keep the existing JPEG conversion before either backend.
        if not image.startswith(b"\xff\xd8"):
            from PIL import Image

            with Image.open(io.BytesIO(image)) as frame:
                output = io.BytesIO()
                frame.convert("RGB").save(output, "JPEG", quality=85)
                image = output.getvalue()

        if self.upstream is None:
            result = self.post(
                "/v1/image-inference-base64",
                {
                    "image_base64": base64.b64encode(image).decode("ascii"),
                    "prompt": prompt,
                    "max_new_tokens": self.config["max_new_tokens"],
                    "temperature": self.config["temperature"],
                    "top_p": self.config["top_p"],
                },
                timeout,
            )
            if result.get("status") != "success":
                logger.warning(
                    "inference failed stamp_ns=%s reason=provider_response", stamp_ns
                )
                raise ValueError(
                    f"VLM service error: {result.get('error', result.get('detail', 'unknown'))}"
                )
        else:
            result = self._direct_request(
                image, prompt, stamp_ns, timeout,
                timing=stages if diagnostics is not None else None,
            )

        if diagnostics is not None:
            fields.update(model=result.get("model"), provider_response_id=result.get("response_id"))
        text = result.get("response")
        if not isinstance(text, str) or not text.strip():
            logger.warning("inference failed stamp_ns=%s reason=invalid_text", stamp_ns)
            raise ValueError("VLM returned empty/invalid text")
        elapsed_ms = (time.perf_counter() - started) * 1000
        logger.info(
            "inference completed stamp_ns=%s image_width=%d image_height=%d elapsed_ms=%.1f",
            stamp_ns,
            *size,
            elapsed_ms,
        )
        return {
            "text": text,
            "elapsed_ms": elapsed_ms,
            "source": source,
            "stamp_ns": stamp_ns,
            "image_size": size,
        }

    def _direct_request(self, image, prompt, stamp_ns, timeout, *, timing=None):
        import cv2
        import numpy as np
        from httpx import ReadTimeout

        started = time.monotonic()
        decoded = cv2.imdecode(np.frombuffer(image, np.uint8), cv2.IMREAD_COLOR)
        if decoded is None:
            raise ValueError("Invalid image")
        if timing is None:
            timing = {}
        timing["decode"] = time.monotonic() - started
        if timeout is not None:
            timeout -= timing["decode"]
        try:
            return self.upstream.infer(decoded, prompt, timing=timing, timeout=timeout)
        except TimeoutError as exc:
            raise ReadTimeout(str(exc)) from None
        finally:
            logger.info(
                "dashscope timing_ms stamp_ns=%s %s",
                stamp_ns,
                " ".join(f"{stage}={seconds * 1000:.1f}" for stage, seconds in timing.items()),
            )

    def health(self):
        if self.upstream is None:
            return {"health": self.get("/health"), "info": self.get("/info")}
        return {
            "health": {"status": "configured", "backend": "dashscope"},
            "info": {"model": self.upstream.model, "device": "cloud"},
        }
