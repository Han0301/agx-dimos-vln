"""D455 visible-object skill. Geometry uses measured depth, not model 3D output."""

import json
import logging
import math
import time
from contextlib import nullcontext
from uuid import uuid4

import numpy as np
from httpx import HTTPError
from scipy.spatial.transform import Rotation

from dimos.plugins.runtime_log import artifact, event, event_context, recording_enabled

logger = logging.getLogger('dimos.plugins.vln_task.preview')

TARGET_PARAMS = {
    "type": "object",
    "properties": {
        "target": {
            "type": "string",
            "description": "Object description with the user's visual attributes, relative position, landmarks, and exclusions.",
        }
    },
    "required": ["target"],
    "additionalProperties": False,
}

LOCATE_TOOL = {
    "type": "function",
    "function": {
        "name": "locate_visible_object",
        "description": "Locate one described object in the current D455 view and estimate its position from aligned depth.",
        "parameters": TARGET_PARAMS,
    },
}

PREVIEW_TOOL = {
    "type": "function",
    "function": {
        "name": "preview_approach",
        "description": "Locate one described object in the current D455 view and preview an approach. Does not move the robot.",
        "parameters": TARGET_PARAMS,
    },
}


def detection_prompt(target):
    return (
        f"Target: {json.dumps(target, ensure_ascii=False)}\n"
        "Find the target in the image. Return only JSON:\n"
        "One candidate: an array with one object containing label, bbox_2d, and appearance_match.\n"
        "bbox_2d: [xmin,ymin,xmax,ymax], normalized to 0–1000, origin at top left.\n"
        "appearance_match: match, mismatch, or uncertain.\n"
        'No candidate: []. Uncertain without a box: [{"status":"uncertain"}].'
    )


def _unique_detection_fields(pairs):
    fields = {}
    for key, value in pairs:
        if key in fields:
            raise ValueError(f"Duplicate detection field: {key}")
        fields[key] = value
    return fields


def parse_detection(text, *, include_fields=False):
    text = text.strip()
    if text.startswith("```") and text.endswith("```"):
        text = text.split("\n", 1)[1][:-3].strip() if "\n" in text else ""
    data = json.loads(text, object_pairs_hook=_unique_detection_fields)
    if data == []:
        return None
    if include_fields and data == [{"status": "uncertain"}]:
        return data[0]
    if not isinstance(data, list) or len(data) != 1 or not isinstance(data[0], dict):
        raise ValueError("Expected exactly one detection or []")
    box = data[0].get("bbox_2d")
    if (
        not isinstance(box, list)
        or len(box) != 4
        or any(
            type(v) not in (float, int) or not math.isfinite(v) or not 0 <= v <= 1000
            for v in box
        )
        or box[0] >= box[2]
        or box[1] >= box[3]
    ):
        raise ValueError("Expected ordered bbox_2d in normalized 0–1000 coordinates")
    if include_fields:
        candidate = data[0]
        verdict = candidate.get("appearance_match")
        if verdict not in ("match", "mismatch", "uncertain"):
            raise ValueError("Expected appearance_match to be match, mismatch, or uncertain")
        return {
            "bbox_2d": box,
            "label": _candidate_text(candidate.get("label"), 120),
            "evidence": _candidate_text(candidate.get("evidence"), 240),
            "appearance_match": verdict,
        }
    return box


def _candidate_text(value, limit):
    if isinstance(value, str) and 0 < len(value.strip()) <= limit:
        return value.strip()
    return None


class TargetDepthError(ValueError):
    pass


def optical_ray(frame, u, v):
    k = np.asarray(frame.k, dtype=float).reshape(3, 3)
    if not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0:
        raise ValueError("Invalid camera intrinsics")
    ray = np.linalg.solve(k, [u, v, 1])
    x, y = ray[:2] / ray[2]
    if any(frame.distortion):
        if frame.distortion_model != "plumb_bob" or len(frame.distortion) != 5:
            raise ValueError("Unsupported nonzero CameraInfo distortion")
        # Invert the standard five-coefficient Brown–Conrady pixel projection.
        k1, k2, p1, p2, k3 = frame.distortion
        xd, yd = x, y
        for _ in range(8):
            r2 = x * x + y * y
            radial = 1 + r2 * (k1 + r2 * (k2 + r2 * k3))
            if not math.isfinite(radial) or radial <= 0:
                raise ValueError("Invalid distortion correction")
            x, y = (
                (xd - 2 * p1 * x * y - p2 * (r2 + 2 * x * x)) / radial,
                (yd - p1 * (r2 + 2 * y * y) - 2 * p2 * x * y) / radial,
            )
    return np.array([1., -x, -y])


def target_and_goal(frame, box, camera, stop_m, *, diagnostics=None):
    height, width = frame.depth.shape
    left, top, right, bottom = np.asarray(box) * [width, height, width, height] / 1000
    u, v = (left + right) / 2, (top + bottom) / 2
    dx, dy = (right - left) * 0.3, (bottom - top) * 0.3
    roi = frame.depth[int(v - dy) : math.ceil(v + dy), int(u - dx) : math.ceil(u + dx)]
    valid = roi[np.isfinite(roi) & (roi > 0)]
    if diagnostics is not None:
        diagnostics.update(
            roi_pixels=int(roi.size), roi_valid_pixels=int(valid.size),
            roi_valid_fraction=valid.size / roi.size if roi.size else 0.,
        )
    if not valid.size:
        raise TargetDepthError("Target box has no usable depth")
    valid_median = np.median(valid)
    inliers = valid[valid <= 2 * valid_median]
    if diagnostics is not None:
        diagnostics.update(
            raw_median_depth_m=float(valid_median), roi_inlier_pixels=int(inliers.size),
            roi_inlier_fraction=inliers.size / roi.size,
        )
    if inliers.size / roi.size < 0.05:
        raise TargetDepthError("Too little valid depth in target box")
    z = float(np.median(inliers))
    if diagnostics is not None:
        diagnostics["median_depth_m"] = z
    if z > 5:
        raise TargetDepthError("Target depth is outside the 5 m localization range")

    optical_in_base = optical_ray(frame, u, v) * z
    p_base = (
        Rotation.from_rotvec([0, camera["pitch"], 0]).apply(optical_in_base)
        + camera["translation"]
    )
    p, q = frame.odom.pose.position, frame.odom.pose.orientation
    origin = np.array([p.x, p.y, p.z])
    if diagnostics is not None:
        diagnostics.update(robot_xyz=origin.tolist(), robot_quat_xyzw=[q.x, q.y, q.z, q.w])
    target = Rotation.from_quat([q.x, q.y, q.z, q.w]).apply(p_base) + origin
    if not np.isfinite(target).all() or p_base[0] <= 0:
        raise ValueError("Target is invalid or behind the base")
    delta = target[:2] - origin[:2]
    distance = float(np.linalg.norm(delta))
    approach = (
        origin[:2] + delta * (max(0, distance - stop_m) / distance)
        if distance
        else origin[:2]
    )
    return target.tolist(), approach.tolist(), z


class ObjectPerception:
    """One observation and target description in; checked localization out."""

    def __init__(self, vlm, camera, stop_m, *, verify=None):
        self.vlm = vlm
        self.camera = camera
        self.stop_m = stop_m
        self.verify = verify

    def detect(
        self, frame, target, remaining, *, observation_ms=None, allow_bad_depth=False,
        stop_m=None, allow_verification=True, purpose=None,
    ):
        det_id = uuid4().hex if recording_enabled() else None
        purpose_fields = {"purpose": purpose} if purpose is not None else {}
        diagnostics = event_context(
            det_id=det_id, rgb_stamp_ns=frame.stamp_ns,
            observation_id=getattr(frame, "observation_id", None), **purpose_fields,
        ) if det_id is not None else nullcontext()
        with diagnostics:
            return self._detect(
                frame, target, remaining, observation_ms=observation_ms,
                allow_bad_depth=allow_bad_depth, stop_m=stop_m,
                allow_verification=allow_verification, det_id=det_id,
            )

    def _detect(
        self, frame, target, remaining, *, observation_ms, allow_bad_depth, stop_m,
        allow_verification, det_id,
    ):
        if stop_m is not None and (
            type(stop_m) not in (float, int) or not math.isfinite(stop_m) or stop_m <= 0
        ):
            raise ValueError("Requested stopping distance must be positive and finite")
        standoff = self.stop_m if stop_m is None else stop_m
        elapsed = {
            "observation_wait": frame.wait_ms,
        }
        if observation_ms is not None:
            elapsed["observation"] = observation_ms
        timeout = min(remaining(), self.vlm.config["timeout_s"])
        try:
            visual = self.vlm.query(
                frame.jpeg,
                detection_prompt(target),
                source="d455",
                stamp_ns=frame.stamp_ns,
                timeout=timeout,
            )
        except (HTTPError, ValueError, RuntimeError, TimeoutError) as exc:
            if det_id is not None:
                path = artifact(f"det-{det_id}.jpg", frame.jpeg, reason="provider_failure")
                event(logger, "detection_result", status="provider_failure",
                      error_type=type(exc).__name__, rgb_artifact=path)
            raise
        remaining()
        elapsed["vlm"] = visual["elapsed_ms"]
        outcome = {
            "status": "not_visible",
            "standoff_m": standoff,
            "observation": {
                "stamp_ns": frame.stamp_ns,
                "depth_gap_ms": frame.depth_gap_ms,
                "odom_gap_ms": frame.odom_gap_ms,
            },
            "timing_ms": elapsed,
        }
        roi_stats = {} if det_id is not None else None

        def report():
            if det_id is None:
                return
            status = outcome["status"]
            match = outcome.get("target_match", {})
            original = outcome.get("appearance_match")
            fields = {}
            if status in {
                "invalid_detection", "verification_unavailable", "target_mismatch",
                "invalid_depth", "invalid_geometry",
            } or original == "uncertain":
                reason = "uncertain_detection" if status == "localized" else status
                fields["rgb_artifact"] = artifact(f"det-{det_id}.jpg", frame.jpeg, reason=reason)
                # Keep a bounded prefix of the original reply.
                raw = visual["text"][:16384]
                fields["response_artifact"] = artifact(
                    f"det-{det_id}.txt", raw, reason=reason,
                    truncated=len(raw) < len(visual["text"]),
                )
            event(
                logger, "detection_result", status=status, label=outcome.get("label"),
                bbox_2d=outcome.get("bbox_2d"), visible_evidence=outcome.get("evidence"),
                detector_verdict=original, final_verdict=match.get("verdict"),
                verification_method=match.get("method"), reason=outcome.get("detection_error"),
                world_frame=outcome.get("world_frame"), target_xyz=outcome.get("target_xyz"),
                goal_xy=outcome.get("goal_xy"), standoff_m=standoff,
                depth_gap_ms=frame.depth_gap_ms, odom_gap_ms=frame.odom_gap_ms,
                timing_ms=dict(elapsed), **roi_stats, **fields,
            )

        try:
            candidate = parse_detection(visual["text"], include_fields=True)
        except ValueError as exc:
            logger.info("detection rejected stamp_ns=%d reason=%s", frame.stamp_ns, exc)
            outcome.update(status="invalid_detection", detection_error=str(exc))
            report()
            return outcome
        if candidate is None:
            logger.info("no target detected stamp_ns=%d", frame.stamp_ns)
            report()
            return outcome
        if candidate.get("status") == "uncertain":
            logger.info("no unique candidate stamp_ns=%d", frame.stamp_ns)
            outcome.update(status="verification_unavailable", detection_error="No unique candidate")
            report()
            return outcome
        outcome.update(candidate)
        box = candidate["bbox_2d"]
        verdict = candidate["appearance_match"]
        method = "detector"
        if verdict == "uncertain" and allow_verification and self.verify is not None:
            method = "visual"
            try:
                verdict, elapsed["verification"] = self.verify(
                    frame, target, box, remaining, include_context=True,
                )
            except (HTTPError, ValueError) as exc:
                logger.info(
                    "visual verification failed stamp_ns=%d error=%s",
                    frame.stamp_ns, type(exc).__name__,
                )
            remaining()
            if verdict not in ("match", "mismatch"):
                verdict = "uncertain"
        outcome["target_match"] = {"verdict": verdict, "method": method}
        logger.info(
            "candidate checked stamp_ns=%d label=%r bbox_2d=%s evidence=%r "
            "verdict=%s method=%s verification_ms=%s",
            frame.stamp_ns, candidate["label"], box, candidate["evidence"],
            verdict, method, elapsed.get("verification"),
        )
        if verdict != "match":
            outcome["status"] = (
                "target_mismatch" if verdict == "mismatch" else "verification_unavailable"
            )
            report()
            return outcome
        started = time.perf_counter()
        try:
            target_xyz, goal, distance = target_and_goal(
                frame, box, self.camera, standoff, diagnostics=roi_stats,
            )
        except TargetDepthError as exc:
            logger.info("target depth invalid stamp_ns=%d reason=%s", frame.stamp_ns, exc)
            outcome.update(status="invalid_depth", detection_error=str(exc))
            report()
            if not allow_bad_depth:
                raise
            return outcome
        except ValueError as exc:
            outcome.update(status="invalid_geometry", detection_error=str(exc))
            report()
            raise
        elapsed["localization"] = (time.perf_counter() - started) * 1000
        outcome.update(
            status="localized",
            depth_m=distance,
            target_xyz=target_xyz,
            goal_xy=goal,
            world_frame=frame.odom.header.frame_id,
        )
        report()
        return outcome
