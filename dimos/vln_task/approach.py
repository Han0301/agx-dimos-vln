"""Refresh an approach goal while handing validated XNavDP plans to the tracker."""

import logging
import math
import threading
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from contextvars import copy_context

from dimos.plugins.path_tracker.messages import decode_result
from dimos.plugins.runtime_log import event
from dimos.plugins.xnavdp.client import XNavDPBusy, XNavDPInferenceError

from .agent import TaskCancelled
from .observation import TRACKING_MAX_AGE_S
from .target_refresh import TargetRefresh

logger = logging.getLogger('dimos.vln_task.approach')


class _Arrived(Exception):
    pass


class ContinuousApproach(TargetRefresh):
    def __init__(
        self, observe, plan, tracker, *, interval_s=0.5, plan_timeout_s=3.0,
        detect=None, refresh_interval_s=1.5,
        last_request_stamp=None, arrival_condition=None,
    ):
        self.observe = observe
        self.plan = plan
        self.tracker = tracker
        self.interval_s = interval_s
        self.plan_timeout_s = plan_timeout_s
        self.last_request_stamp = last_request_stamp
        self.arrival_condition = arrival_condition
        self.detect = detect
        self.refresh_interval_s = refresh_interval_s
        self.worker = (
            ThreadPoolExecutor(max_workers=1, thread_name_prefix="approach-vlm")
            if detect is not None else None
        )
        self.lock = threading.Lock()
        self.active = None
        self.guarded = False
        self.standoff_active = False

    def _clear(self, reason):
        self.tracker.clear_plan(reason)
        if self.standoff_active:
            self.tracker.clear_standoff()
            self.standoff_active = False
        if self.guarded:
            self.tracker.set_motion_guard(None)
            self.guarded = False
        self.active = None

    def close(self):
        if self.worker is not None:
            self.worker.shutdown(wait=False, cancel_futures=True)

    def cancel(self, cancel_event):
        with self.lock:
            if self.active is cancel_event:
                event(logger, "approach_cancel_requested")
                cancel_event.set()
                self._clear("task_cancelled")

    def _wait_for_replan(self, deadline, check, cancel):
        if self.arrival_condition is None:
            cancel.wait(min(max(0.0, deadline - time.monotonic()), check()))
            return
        # The tracker notifies this condition when fresh odometry arrives.
        with self.arrival_condition:
            while True:
                budget = check()
                delay = deadline - time.monotonic()
                if delay <= 0:
                    return
                self.arrival_condition.wait(timeout=min(delay, budget, 0.1))

    def __call__(
        self, outcome, remaining, cancel, on_progress=None,
        *, target=None, accept_path=None, motion_guard=None, initial_observation=None,
    ):
        goal = tuple(outcome["goal_xy"])
        world_frame = outcome["world_frame"]
        last_stamp = outcome["observation"]["stamp_ns"]
        updates = points = 0
        timing = dict(outcome["timing_ms"])
        planning_total = 0.0
        end_reason = "task_ended"
        failure = None
        started_at = time.monotonic()
        leg = outcome.get("exploration_leg", {})
        refreshed = deque(maxlen=1)
        refresh_stopped = threading.Event()
        refresh = None

        def check():
            if cancel.is_set():
                raise TaskCancelled("Task cancelled; late results discarded")
            budget = remaining()
            if (self.standoff_active or not refreshed) and self.tracker.reached(goal, world_frame):
                raise _Arrived
            return budget

        def progress(status):
            if on_progress is not None:
                on_progress({"status": status, "path_updates": updates})

        with self.lock:
            remaining()
            if cancel.is_set():
                raise TaskCancelled("Task cancelled; late results discarded")
            if self.active is not None:
                raise RuntimeError("Another approach is active")
            self.active = cancel
            if "standoff_m" in outcome and "target_xyz" in outcome:
                self.tracker.set_standoff(outcome["target_xyz"][:2], outcome["standoff_m"], world_frame)
                self.standoff_active = True
                logger.info("approach standoff_m=%.3f", outcome["standoff_m"])
            if motion_guard is not None:
                self.tracker.set_motion_guard(motion_guard)
                self.guarded = True
        event(
            logger, "approach_started", **leg, source_stamp_ns=last_stamp,
            goal_xy=goal, world_frame=world_frame,
            target_xyz=outcome.get("target_xyz"), standoff_m=outcome.get("standoff_m"),
        )
        try:
            progress("planning")
            if self.worker is not None and target is not None:
                context = copy_context()
                refresh = self.worker.submit(
                    context.run, self._refresh_targets, target, outcome, remaining, cancel,
                    refresh_stopped, refreshed,
                )
            while True:
                if self.standoff_active:
                    check()
                try:
                    detection = refreshed.popleft()
                except IndexError:
                    pass
                else:
                    with self.lock:
                        if self.standoff_active:
                            check()
                            if not self.tracker.set_standoff(
                                detection["target_xyz"][:2], outcome["standoff_m"], world_frame
                            ):
                                raise _Arrived
                    outcome = {**outcome, **detection}
                    goal = tuple(detection["goal_xy"])
                    last_stamp = max(last_stamp, detection["observation"]["stamp_ns"])
                check()
                started = time.monotonic()
                phase = "observation"
                request_timing = {}
                try:
                    seed, initial_observation = initial_observation, None
                    last_requested = (
                        self.last_request_stamp() or 0
                        if seed is not None and self.last_request_stamp is not None else 0
                    )
                    reused_seed = (
                        seed is not None
                        and seed.stamp_ns == last_stamp
                        and seed.stamp_ns > last_requested
                        and seed.odom.header.frame_id == world_frame
                        and 0 <= started - seed.received_at <= TRACKING_MAX_AGE_S
                    )
                    if reused_seed:
                        frame = seed
                    else:
                        after_stamp = max(last_stamp, last_requested)
                        frame = self.observe(
                            check, after_stamp_ns=after_stamp, max_age_s=TRACKING_MAX_AGE_S,
                        )
                        if frame.stamp_ns <= after_stamp:
                            raise TimeoutError("No newer RGB frame for this goal")
                    last_stamp = frame.stamp_ns
                    if frame.odom.header.frame_id != world_frame:
                        raise RuntimeError("Odometry frame changed during approach")
                    plan_started = time.monotonic()
                    plan_deadline = plan_started + self.plan_timeout_s

                    def plan_check():
                        budget = check()
                        if time.monotonic() >= plan_deadline:
                            raise TimeoutError("XNavDP update deadline exceeded")
                        return min(budget, plan_deadline - time.monotonic())

                    event(
                        logger, "approach_planning_started", **leg, request_stamp_ns=last_stamp,
                        goal_xy=goal, world_frame=world_frame,
                        observation_id=getattr(frame, "observation_id", None),
                        depth_stamp_ns=getattr(frame, "depth_stamp_ns", None),
                        odom_stamp_ns=getattr(frame, "odom_stamp_ns", None),
                        depth_gap_ms=getattr(frame, "depth_gap_ms", None),
                        odom_gap_ms=getattr(frame, "odom_gap_ms", None),
                        observation_wait_ms=(plan_started - started) * 1000,
                        reused_initial_observation=reused_seed,
                    )
                    phase = "planner"
                    try:
                        data, seconds, path_points = self.plan(
                            frame, goal, plan_check, request_timing
                        )
                        phase = "decode_result"
                        result = decode_result(data)
                        if result.stamp_ns != last_stamp or result.world_frame != world_frame:
                            raise ValueError("XNavDP result belongs to another observation")
                    except (ValueError, KeyError, TypeError, IndexError) as exc:
                        logger.warning(
                            "invalid path discarded stamp_ns=%d error_type=%s",
                            last_stamp, type(exc).__name__,
                        )
                        event(logger, "approach_result_invalid", **leg,
                              request_stamp_ns=last_stamp, stage=phase,
                              reason=str(exc)[:300], error_type=type(exc).__name__)
                        raise TimeoutError("Invalid XNavDP update") from exc
                    finally:
                        planning_total += (time.monotonic() - plan_started) * 1000
                    phase = "path_acceptance"
                    allowed = accept_path is None or accept_path(frame, result, plan_check)
                    with self.lock:
                        plan_check()
                        if allowed:
                            accepted = self.tracker.submit(result)
                        else:
                            logger.warning("Path leaves mapped free space stamp_ns=%d", last_stamp)
                            self.tracker.clear_plan("exploration_path_blocked")
                            accepted = False
                    event(
                        logger, "approach_planning_finished", **leg,
                        request_stamp_ns=last_stamp, goal_xy=goal, world_frame=world_frame,
                        outcome="accepted" if accepted else "rejected",
                        reason=None if accepted else ("tracker_rejected" if allowed else "blocked_path"),
                        path_points=path_points, timing_ms=request_timing,
                        elapsed_ms=(time.monotonic() - started) * 1000,
                    )
                    if accepted:
                        updates += 1
                        points = path_points
                        timing.update(request_timing, xnavdp_roundtrip=seconds * 1000)
                        logger.info(
                            "path update accepted update=%d request_stamp_ns=%d",
                            updates, last_stamp,
                        )
                        progress("tracking")
                    else:
                        progress("replanning")
                except (TimeoutError, XNavDPBusy, XNavDPInferenceError) as exc:
                    event(
                        logger, "approach_planning_finished", **leg, request_stamp_ns=last_stamp,
                        goal_xy=goal, world_frame=world_frame, outcome="failed", stage=phase,
                        error_type=type(exc).__name__, reason=str(exc)[:300],
                        timing_ms=request_timing, elapsed_ms=(time.monotonic() - started) * 1000,
                    )
                    check()
                    logger.info(
                        "waiting for a new path stamp_ns=%d reason=%s",
                        last_stamp, type(exc).__name__,
                    )
                    progress("replanning")
                self._wait_for_replan(started + self.interval_s, check, cancel)
        except _Arrived:
            end_reason = "goal_reached"
        except Exception as exc:
            failure = type(exc).__name__
            raise
        finally:
            refresh_stopped.set()
            if refresh is not None:
                refresh.cancel()
            reason = "task_cancelled" if cancel.is_set() else end_reason
            with self.lock:
                if self.active is cancel:
                    self._clear(reason)
            event(
                logger, "approach_finished", **leg, reason=reason, error_type=failure,
                last_observation_stamp_ns=last_stamp, goal_xy=goal, world_frame=world_frame,
                path_updates=updates, elapsed_ms=(time.monotonic() - started_at) * 1000,
            )
        if cancel.is_set():
            raise TaskCancelled("Task cancelled; late results discarded")
        progress("arrived")
        timing["planning_total"] = planning_total
        return {
            **outcome, "status": "arrived", "path_updates": updates,
            "path_points": points, "timing_ms": timing,
        }
