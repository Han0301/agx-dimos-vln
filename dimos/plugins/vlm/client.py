"""Explicit-image VLM using DashScope directly or the compatible HTTP protocol."""

import base64
import io
import logging
import time
from pathlib import Path
from uuid import uuid4

from httpx import HTTPError, HTTPStatusError

from dimos.plugins._agent_common import HTTPModel, validate_image
from dimos.plugins.runtime_log import diagnostic_artifact, event, recording_enabled

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

    @property
    def supports_multi_image(self):
        return self.upstream is not None

    def query(
        self, image, prompt, *, source="explicit_image", stamp_ns=None, timeout=None
    ):
        return self._query_with_diagnostics((image,), prompt, source, stamp_ns, timeout)

    def query_images(
        self, images, prompt, *, source="explicit_images", stamp_ns=None, timeout=None
    ):
        """Send two separate images in order and return one model response."""
        if not isinstance(images, (list, tuple)) or len(images) != 2:
            raise ValueError("VLM query_images expects two ordered images")
        if not self.supports_multi_image:
            raise ValueError("The legacy HTTP VLM backend accepts one image per request")
        return self._query_with_diagnostics(tuple(images), prompt, source, stamp_ns, timeout)

    def _query_with_diagnostics(self, images, prompt, source, stamp_ns, timeout):
        if not recording_enabled():
            return self._query(images, prompt, source, stamp_ns, timeout, None)
        started = time.perf_counter()
        stages = {}
        image = images[-1]
        fields = {
            "call_id": uuid4().hex, "source": source, "rgb_stamp_ns": stamp_ns,
            "backend": self.config.get("backend", "http"), "timeout_s": timeout,
            "image_bytes": len(image) if isinstance(image, bytes) else None,
            "input_image_artifact": None, "input_image_archive_status": "not_prepared",
        }
        try:
            result = self._query(images, prompt, source, stamp_ns, timeout, (fields, stages))
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

    def _query(self, images, prompt, source, stamp_ns, timeout, diagnostics):
        started = time.perf_counter()
        sizes = [validate_image(image) for image in images]
        size = sizes[-1]
        multi_image = len(images) > 1
        archive_input = None
        if diagnostics is not None:
            fields, stages = diagnostics
            fields["image_size"] = size
            archived = []
            if multi_image:
                fields.update(image_sizes=sizes, input_images=archived)

            def archive_input(jpeg):
                index = len(archived)
                suffix = f"-{index + 1}" if multi_image else ""
                path = diagnostic_artifact(
                    f"vlm-input-{fields['call_id']}{suffix}.jpg", jpeg,
                    call_id=fields["call_id"], source=source, rgb_stamp_ns=stamp_ns,
                    backend=fields["backend"], kind="vlm_input_image", image_index=index + 1,
                )
                status = "queued" if path else "not_queued"
                archived.append(
                    {"index": index + 1, "artifact": path, "bytes": len(jpeg), "status": status}
                )
                if index == len(images) - 1:
                    fields.update(
                        input_image_artifact=path, input_image_bytes=len(jpeg),
                        input_image_archive_status=status,
                    )
        if not isinstance(prompt, str) or not prompt.strip():
            raise ValueError("VLM prompt must be nonempty")
        if stamp_ns is not None and (type(stamp_ns) is not int or stamp_ns < 0):
            raise ValueError("stamp_ns must be a nonnegative integer")
        images = tuple(_as_jpeg(image) for image in images)

        if self.upstream is None:
            image = images[0]
            if archive_input is not None:
                archive_input(image)
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
                images, prompt, stamp_ns, timeout,
                timing=stages if diagnostics is not None else None,
                archive_input=archive_input,
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
        response = {
            "text": text,
            "elapsed_ms": elapsed_ms,
            "source": source,
            "stamp_ns": stamp_ns,
            "image_size": size,
        }
        if multi_image:
            response["image_sizes"] = sizes
        return response

    def _direct_request(self, images, prompt, stamp_ns, timeout, *, timing=None, archive_input=None):
        import cv2
        import numpy as np
        from httpx import ReadTimeout

        started = time.monotonic()
        decoded = tuple(
            cv2.imdecode(np.frombuffer(image, np.uint8), cv2.IMREAD_COLOR) for image in images
        )
        if any(image is None for image in decoded):
            raise ValueError("Invalid image")
        if timing is None:
            timing = {}
        timing["decode"] = time.monotonic() - started
        if timeout is not None:
            timeout -= timing["decode"]
        try:
            infer = self.upstream.infer if len(decoded) == 1 else self.upstream.infer_images
            return infer(
                decoded[0] if len(decoded) == 1 else decoded, prompt,
                timing=timing, timeout=timeout, archive_input=archive_input,
            )
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


def _as_jpeg(image):
    if image.startswith(b"\xff\xd8"):
        return image
    from PIL import Image

    with Image.open(io.BytesIO(image)) as frame:
        output = io.BytesIO()
        frame.convert("RGB").save(output, "JPEG", quality=85)
        return output.getvalue()
