"""Compose one perception result with optional path and speed preview."""

import json
import logging
import math
import time

from dimos.plugins.runtime_log import event_context

# Existing import paths keep exposing the same geometry and tool contracts.
from .perception import (
    LOCATE_TOOL, PREVIEW_TOOL, TARGET_PARAMS, ObjectPerception, TargetDepthError,
    detection_prompt, optical_ray, parse_detection, target_and_goal,
)

logger = logging.getLogger("dimos.plugins.vln_task.preview")


class NavigationPreview(ObjectPerception):
    def __init__(self, vlm, snapshot, plan, preview, camera, stop_m, *, verify=None):
        super().__init__(vlm, camera, stop_m, verify=verify)
        self.snapshot = snapshot
        self.plan = plan
        self.preview = preview

    def __call__(self, target, remaining, *, plan_path=True, stop_m=None):
        if not isinstance(target, str) or not target.strip() or len(target) > 1000:
            raise ValueError("Expected a nonempty object description")
        start = time.perf_counter()
        stop_options = {"stop_m": stop_m} if stop_m is not None else {}
        with event_context(purpose="preview_det" if plan_path else "locate_det"):
            frame = self.snapshot(remaining)
            outcome = self.detect(
                frame,
                target,
                remaining,
                observation_ms=(time.perf_counter() - start) * 1000,
                **stop_options,
            )
        if not plan_path or self.plan is None or outcome["status"] != "localized":
            return outcome
        return self.plan_detection(frame, outcome, remaining)

    def plan_detection(self, frame, outcome, remaining):
        outcome = {**outcome, "timing_ms": dict(outcome["timing_ms"])}
        elapsed = outcome["timing_ms"]
        goal = outcome["goal_xy"]
        origin = frame.odom.pose.position
        if math.hypot(goal[0] - origin.x, goal[1] - origin.y) < 1e-6:
            outcome["status"] = "within_standoff"
            return outcome
        started = time.perf_counter()
        data, seconds, points = self.plan(frame, goal, remaining, elapsed)
        elapsed["planning_total"] = (time.perf_counter() - started) * 1000
        elapsed["xnavdp_roundtrip"] = seconds * 1000
        remaining()
        if json.loads(data)["request_stamp_ns"] != frame.stamp_ns:
            raise ValueError("XNavDP result belongs to another observation")
        started = time.perf_counter()
        command = self.preview(data)
        elapsed["tracker"] = (time.perf_counter() - started) * 1000
        remaining()
        outcome.update(
            status="preview_ready", path_points=points, speed_preview=command
        )
        return outcome
