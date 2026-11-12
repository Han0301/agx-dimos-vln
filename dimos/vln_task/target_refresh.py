"""Task-level visual target refresh used during an approach."""

import logging
import math
import time

from httpx import HTTPError
from dimos.plugins.runtime_log import event
from .agent import TaskCancelled
from .observation import TRACKING_MAX_AGE_S

logger = logging.getLogger("dimos.vln_task.approach")
_MAX_REFRESH_DISTANCE_M = 1.0


class TargetRefresh:
    """Refresh policy mixed into the task controller; owns no model or motion IO."""

    def _refresh_targets(self, target, initial, remaining, cancel, stopped, updates):
        stamp = initial["observation"]["stamp_ns"]
        world_frame = initial["world_frame"]
        last_xyz = initial["target_xyz"]
        initial_xy = tuple(last_xyz[:2])
        standoff = initial.get("standoff_m")
        options = {"stop_m": standoff} if standoff is not None else {}

        def check():
            if stopped.is_set() or cancel.is_set():
                raise TaskCancelled("Approach target refresh ended")
            return remaining()

        next_check = time.monotonic() + self.refresh_interval_s
        while not stopped.wait(max(0.0, next_check - time.monotonic())):
            next_check = time.monotonic() + self.refresh_interval_s
            try:
                frame = self.observe(
                    check, after_stamp_ns=stamp, max_age_s=TRACKING_MAX_AGE_S
                )
                if frame.stamp_ns <= stamp:
                    continue
                stamp = frame.stamp_ns
                if frame.odom.header.frame_id != world_frame:
                    event(logger, "target_refresh", outcome="rejected", reason="world_frame_changed",
                          stamp_ns=stamp, expected_world_frame=world_frame,
                          world_frame=frame.odom.header.frame_id)
                    continue
                next_check = time.monotonic() + self.refresh_interval_s
                result = self.detect(frame, target, check, **options)
                check()
            except TaskCancelled:
                return
            except (HTTPError, TimeoutError, ValueError) as exc:
                logger.info("target refresh skipped reason=%s", type(exc).__name__)
                event(logger, "target_refresh", outcome="failed", stamp_ns=stamp,
                      error_type=type(exc).__name__)
                continue

            if result["status"] != "localized" or result["world_frame"] != world_frame:
                event(logger, "target_refresh", outcome="rejected", stamp_ns=stamp,
                      reason=result["status"] if result["status"] != "localized" else "world_frame_changed")
                continue
            distance = math.dist(result["target_xyz"][:2], initial_xy)
            if distance > _MAX_REFRESH_DISTANCE_M:
                logger.info("target refresh rejected stamp_ns=%d distance_from_initial_xy_m=%.3f",
                            stamp, distance)
                event(logger, "target_refresh", outcome="rejected", reason="too_far_from_initial_target",
                      stamp_ns=stamp, distance_from_initial_xy_m=distance)
                continue
            shift = math.dist(result["target_xyz"], last_xyz)
            last_xyz = result["target_xyz"]
            updates.append(result)
            logger.info("target refreshed stamp_ns=%d goal_xy=%s", stamp, result["goal_xy"])
            event(logger, "target_refresh", outcome="accepted", stamp_ns=stamp,
                  shift_m=shift, target_xyz=last_xyz, goal_xy=result["goal_xy"])
