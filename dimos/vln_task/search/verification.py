import io
import json
import logging
import math
from contextlib import nullcontext
from uuid import uuid4

from httpx import HTTPError
from PIL import Image, ImageDraw

from dimos.plugins.runtime_log import artifact, diagnostic_artifact, event, event_context, recording_enabled

logger = logging.getLogger('dimos.plugins.active_search.verification')


def verification_image(jpeg, box, *, include_context=False):
    with Image.open(io.BytesIO(jpeg)) as source:
        image = source.convert("RGB")
    x0, y0, x1, y1 = [v * n / 1000 for v, n in zip(box, image.size * 2)]
    x0, y0, x1, y1 = math.floor(x0), math.floor(y0), math.ceil(x1), math.ceil(y1)
    if include_context:
        crop = context_crop(image, (x0, y0, x1, y1))
    else:
        crop = image.crop((x0, y0, x1, y1))
        crop.thumbnail((192, 330))
        if max(crop.size) < 240:
            scale = min(192 / crop.width, 330 / crop.height, 3)
            crop = crop.resize(
                (round(crop.width * scale), round(crop.height * scale)),
                Image.Resampling.LANCZOS,
            )
    output = io.BytesIO()
    crop.save(output, "JPEG", quality=88)
    return output.getvalue()


def context_crop(image, box):
    x0, y0, x1, y1 = box
    width, height = x1 - x0, y1 - y0
    left = max(0, x0 - math.ceil(0.30 * width))
    top = max(0, y0 - math.ceil(0.70 * height))
    right = min(image.width, x1 + math.ceil(0.30 * width))
    bottom = min(image.height, y1 + math.ceil(0.25 * height))
    crop = image.crop((left, top, right, bottom))
    scale = min(320 / crop.width, 320 / crop.height, 4)
    crop = crop.resize(
        (max(1, round(crop.width * scale)), max(1, round(crop.height * scale))),
        Image.Resampling.LANCZOS,
    )
    ImageDraw.Draw(crop).rectangle(
        (
            (x0 - left) * scale,
            (y0 - top) * scale,
            (x1 - left) * scale,
            (y1 - top) * scale,
        ),
        outline=(255, 32, 225),
        width=3,
    )
    return crop


def verification_prompt(target, *, include_context=False):
    subject = "the object inside the magenta box" if include_context else "the object in the crop"
    return (
        f"Does {subject} match {json.dumps(target, ensure_ascii=False)}? "
        'Answer "yes" only when visible evidence supports the category and every requested attribute. '
        'Check any requested color on the object itself. Use context to check requested spatial relationships. '
        'If the category or a required attribute is unclear or conflicts with the target, answer "no". '
        'Return only {"match":"yes"} or {"match":"no"}.'
    )


def verify_candidate(vlm, frame, target, box, remaining, *, include_context=False):
    source = "d455_search_context" if include_context else "d455_search_verification"
    image = verification_image(frame.jpeg, box, include_context=include_context)
    verification_id = uuid4().hex if recording_enabled() else None
    fields = {
        "verification_id": verification_id, "source": source,
        "observation_id": getattr(frame, "observation_id", None),
        "rgb_stamp_ns": frame.stamp_ns, "bbox_2d": box, "target": target,
    } if verification_id is not None else {}
    crop_path = None
    diagnostics = (
        event_context(purpose="visual_verification", verification_id=verification_id)
        if verification_id is not None else nullcontext()
    )
    with diagnostics:
        timeout = min(remaining(), vlm.config["timeout_s"])
        if verification_id is not None:
            crop_path = diagnostic_artifact(
                f"verify-{verification_id}-crop.jpg", image, kind="vlm_verification_crop", **fields,
            )
        try:
            result = vlm.query(
                image, verification_prompt(target, include_context=include_context),
                source=source, stamp_ns=frame.stamp_ns, timeout=timeout,
            )
        except (HTTPError, ValueError, RuntimeError, TimeoutError) as exc:
            if verification_id is not None:
                path = crop_path or artifact(
                    f"verify-{verification_id}.jpg", image, reason="provider_failure", **fields,
                )
                event(logger, "visual_verification_result", **fields, verdict="provider_failure",
                      error_type=type(exc).__name__, crop_artifact=path)
            raise
    text = result["text"].strip()
    if text.startswith("```") and text.endswith("```"):
        text = text.partition("\n")[2][:-3].strip()
    try:
        response_fields = json.loads(text, object_pairs_hook=list)
    except ValueError:
        response_fields = None
    verdict = "invalid_verification"
    if response_fields == [("match", "yes")]:
        verdict = "match"
    elif response_fields == [("match", "no")]:
        verdict = "mismatch"
    if verification_id is not None:
        raw = result["text"][:16384]
        truncated = len(raw) < len(result["text"])
        response_path = diagnostic_artifact(
            f"verify-{verification_id}-response.txt", raw, kind="vlm_verification_response",
            truncated=truncated, verdict=verdict, **fields,
        )
        evidence = {}
        if verdict != "match":
            evidence["crop_artifact"] = crop_path or artifact(
                f"verify-{verification_id}.jpg", image, reason=verdict, **fields,
            )
            evidence["response_artifact"] = response_path or artifact(
                f"verify-{verification_id}.txt", raw, reason=verdict, truncated=truncated, **fields,
            )
        event(logger, "visual_verification_result", **fields, verdict=verdict,
              elapsed_ms=result["elapsed_ms"], crop_diagnostic_artifact=crop_path,
              vlm_response_artifact=response_path, vlm_response_truncated=truncated, **evidence)
    return verdict, result["elapsed_ms"]
