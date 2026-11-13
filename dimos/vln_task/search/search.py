import logging
import math
import time

from dimos.plugins.runtime_log import event

logger = logging.getLogger('dimos.plugins.active_search.search')

SCAN_STATUSES = frozenset({
    "not_visible", "target_mismatch", "verification_unavailable", "invalid_detection",
})
SEARCH_SEED_STATUSES = SCAN_STATUSES | {"invalid_depth", "provider_failure"}


class ExplorationUnavailable(RuntimeError):
    """No safe exploration step can be executed."""


def viewpoint_changed(previous, current, settings):
    x, y, yaw = previous
    new_x, new_y, new_yaw = current
    return math.hypot(new_x - x, new_y - y) >= settings["min_translation_m"] or abs(
        math.remainder(new_yaw - yaw, math.tau)
    ) >= math.radians(settings["min_rotation_deg"])


class ActiveSearch:
    def __init__(self, observe, detect, plan):
        self.observe = observe
        self.detect = detect
        self.plan = plan

    def __call__(
        self, target, remaining, on_progress=None, *,
        next_view=None, initial_frame=None, initial_detection=None, stop_m=None,
        toward_target=None, on_detection=None, exhausted_reason="sweep_complete",
        retry_view=None, recover_memory=None, take_detection=None,
    ):
        """Find the object, optionally obtaining a new stopped view by exploration.

        next_view(previous, budget, on_progress) must clear motion before returning
        a newer observation. It returns None when exploration is exhausted.
        toward_target returns a stopped view after following a box bearing, or
        None to resume the scan.
        """
        if not isinstance(target, str) or not target.strip() or len(target) > 1000:
            raise ValueError("Expected a nonempty object description")
        if initial_detection is not None and (
            initial_frame is None
            or not isinstance(initial_detection, dict)
            or initial_detection.get("status") not in SEARCH_SEED_STATUSES
        ):
            raise ValueError("Expected initial detection eligible for exploration and its observation")
        if stop_m is None and initial_detection is not None:
            stop_m = initial_detection.get("standoff_m")
        detection_options = {"stop_m": stop_m} if stop_m is not None else {}
        started = time.monotonic()
        attempts = []
        frame = initial_frame
        if frame is not None:
            attempts.append({
                **(initial_detection if initial_detection is not None else {"status": "not_visible"}),
                "stamp_ns": frame.stamp_ns,
            })
        terminal_status = "search_exhausted"
        explore = frame is not None and next_view is not None
        depth_failure = initial_detection if (
            initial_detection is not None and initial_detection["status"] == "invalid_depth"
        ) else None
        logger.info("search started")
        event(
            logger, "search_started", target=target,
            live_motion=next_view is not None,
            source_stamp_ns=frame.stamp_ns if frame is not None else None,
            seed_status=initial_detection.get("status") if initial_detection else None,
        )

        def emit(status, **fields):
            additional_attempts = max(0, len(attempts) - 1)
            event(logger, "search_stage", stage=status, attempts=additional_attempts,
                  source_stamp_ns=frame.stamp_ns if frame is not None else None, **fields)
            if on_progress is not None:
                on_progress(
                    {"status": status, "attempts": additional_attempts, **fields}
                )

        if on_detection is not None and initial_detection is not None:
            on_detection(initial_frame, initial_detection)

        provider_failed = initial_detection is not None and initial_detection["status"] == "provider_failure"
        while True:
            try:
                if provider_failed:
                    next_frame = recover_memory(frame, remaining, on_progress) if recover_memory else None
                    if next_frame is None:
                        next_frame = (retry_view or self.observe)(frame, remaining)
                    explore = False
                elif explore:
                    remaining()
                    next_frame = None
                    if depth_failure is not None and toward_target is not None:
                        emit("exploring_toward_target")
                        next_frame = toward_target(frame, depth_failure, remaining)
                        remaining()
                    if next_frame is None:
                        if on_detection is None:
                            emit("exploring")
                        next_frame = next_view(frame, remaining, on_progress)
                        remaining()
                        if next_frame is None:
                            reason = exhausted_reason
                            break
                    depth_failure = None
                    if on_detection is None:
                        emit("searching")
                    explore = False
                else:
                    emit("searching")
                    next_frame = self.observe(frame, remaining)
            except ExplorationUnavailable as exc:
                terminal_status = "exploration_unavailable"
                reason = str(exc) or terminal_status
                break
            except TimeoutError:
                if provider_failed:
                    terminal_status = reason = "provider_failure"
                else:
                    reason = "no_new_view" if frame is not None else "no_observation"
                break
            if frame is not None and next_frame.stamp_ns <= frame.stamp_ns:
                raise ValueError("Active search requires a newer observation")
            frame = next_frame

            if on_detection is not None:
                emit("detecting_with_memory")
            detection = take_detection(frame) if take_detection is not None else None
            if detection is None:
                detection = self.detect(frame, target, remaining, **detection_options)
            remaining()
            attempt = {"stamp_ns": frame.stamp_ns, **detection}
            attempts.append(attempt)
            logger.info(
                "view checked attempt=%d stamp_ns=%d status=%s",
                len(attempts),
                frame.stamp_ns,
                detection["status"],
            )
            event(
                logger, "search_detection", attempt=len(attempts), stamp_ns=frame.stamp_ns,
                observation_id=getattr(frame, "observation_id", None),
                outcome=detection["status"],
                timing_ms=detection.get("timing_ms"),
            )
            if on_detection is not None:
                on_detection(frame, detection)
            status = detection["status"]
            if status == "provider_failure":
                if provider_failed:
                    terminal_status = reason = status
                    break
                provider_failed = True
                continue
            provider_failed = False
            if status != "localized":
                if next_view is not None:
                    if status == "invalid_depth":
                        depth_failure = detection
                    elif status not in SCAN_STATUSES:
                        terminal_status = reason = status
                        break
                    explore = True
                continue
            remaining()
            emit("confirmed")
            search = {
                "reason": "confirmed",
                "attempts": attempts,
                "elapsed_ms": (time.monotonic() - started) * 1000,
            }
            logger.info(
                "search confirmed attempts=%d elapsed_ms=%.1f",
                len(attempts),
                search["elapsed_ms"],
            )
            result = self.plan(frame, detection, remaining)
            event(logger, "search_finished", outcome=result.get("status"), reason="confirmed",
                  attempts=len(attempts), elapsed_ms=search["elapsed_ms"], stamp_ns=frame.stamp_ns)
            return {**result, "search": search}

        remaining()
        emit(terminal_status, **({"reason": reason} if on_detection is not None else {}))
        elapsed_ms = (time.monotonic() - started) * 1000
        logger.info(
            "search ended status=%s reason=%s attempts=%d elapsed_ms=%.1f",
            terminal_status,
            reason,
            len(attempts),
            elapsed_ms,
        )
        event(logger, "search_finished", outcome=terminal_status, reason=reason,
              attempts=len(attempts), elapsed_ms=elapsed_ms)
        return {
            "status": terminal_status,
            "search": {
                "reason": reason,
                "attempts": attempts,
                "elapsed_ms": elapsed_ms,
            },
        }
