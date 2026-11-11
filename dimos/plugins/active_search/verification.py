import io
import json
import logging
import math
from uuid import uuid4

from PIL import Image, ImageDraw

from dimos.plugins.runtime_log import artifact, event, event_context, recording_enabled

logger = logging.getLogger(__name__)


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
        'Return only {"match":"yes"} or {"match":"no"}.'
    )


def verify_candidate(vlm, frame, target, box, remaining, *, include_context=False):
    source = "d455_search_context" if include_context else "d455_search_verification"
    image = verification_image(frame.jpeg, box, include_context=include_context)
    verification_id = uuid4().hex if recording_enabled() else None
    with event_context(purpose="visual_verification", verification_id=verification_id):
        result = vlm.query(
            image, verification_prompt(target, include_context=include_context),
            source=source, stamp_ns=frame.stamp_ns,
            timeout=min(remaining(), vlm.config["timeout_s"]),
        )
    text = result["text"].strip()
    if text.startswith("```") and text.endswith("```"):
        text = text.partition("\n")[2][:-3].strip()
    try:
        fields = json.loads(text, object_pairs_hook=list)
    except ValueError:
        fields = None
    verdict = "invalid_verification"
    if fields == [("match", "yes")]:
        verdict = "match"
    elif fields == [("match", "no")]:
        verdict = "mismatch"
    if verification_id is not None:
        evidence = {}
        if verdict != "match":
            evidence["crop_artifact"] = artifact(
                f"verify-{verification_id}.jpg", image, verification_id=verification_id,
                rgb_stamp_ns=frame.stamp_ns, reason=verdict,
            )
            evidence["response_artifact"] = artifact(
                f"verify-{verification_id}.txt", result["text"][:16384],
                verification_id=verification_id, rgb_stamp_ns=frame.stamp_ns, reason=verdict,
                truncated=len(result["text"]) > 16384,
            )
        event(logger, "visual_verification_result", verification_id=verification_id,
              source=source, rgb_stamp_ns=frame.stamp_ns, bbox_2d=box,
              verdict=verdict, elapsed_ms=result["elapsed_ms"], **evidence)
    return verdict, result["elapsed_ms"]
