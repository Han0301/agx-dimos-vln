"""Select short exploration goals and execute them through the VLN motion stack."""

import math
import time
import threading
from contextlib import contextmanager, nullcontext
from uuid import uuid4

from dimos.plugins._agent_common import TaskCancelled
from dimos.plugins.path_tracker.messages import odom_pose
from dimos.plugins.path_tracker.tracker import GOAL_TOLERANCE_M, TURN_TOLERANCE_RAD
from dimos.plugins.runtime_log import diagnostic_artifact, event, event_context, recording_enabled

from ..observation import TRACKING_MAX_AGE_S
from .exploration import DirectedExploration, RotationSweep, logger, observe_stopped
from .search import SEARCH_SEED_STATUSES, ExplorationUnavailable

_QUERY_REQUIRED = object()
_VIEW_MOTION_BUDGET_S = 15.0
_MEMORY_QUERY_INTERVAL = 5
_MEMORY_MIN_SIMILARITY = 0.20
_RECORDED_DETECTIONS = {
    "localized", "not_visible", "target_mismatch", "invalid_depth", "invalid_detection",
}


class _TargetDetected(Exception):
    pass


class MemoryExploration(DirectedExploration):
    """Search from recalled viewpoints and reachable frontiers."""

    def __init__(
        self, observe, newest_stamp, approach, mapper, planner, cancel, *,
        turn, query_memory=None, prefetched_memory=None, visualization=None, cloud=None, settle_s=0.15,
    ):
        super().__init__(
            observe, newest_stamp, approach, mapper, planner, cancel,
            cloud=cloud, settle_s=settle_s,
        )
        self.query_memory = query_memory
        self.prefetched_memory = prefetched_memory
        self.start_motion_detection = None
        self.motion_detection = None
        self.observed_poses = []
        self.recalled_ids = set()
        self.checked_views = 0
        self.turn = turn
        self.visualization = visualization
        self.pending_view = None
        self.last_frontier_failure = None
        self.latest_map = None
        self.world_frame = None

    def run_search(self, search, target, budget, progress, frame, detection, *, camera, memory_target=None, rotation_fallback=False, retry_view=None):
        recording = recording_enabled()
        started = time.monotonic() if recording else None
        next_query_at = 0
        sweep = None
        lookup_target = target if memory_target is None else memory_target

        def recall(previous, remaining, on_progress, trigger):
            if not recording:
                return self._recall_view(lookup_target, previous, remaining, on_progress)
            with event_context(memory_trigger=trigger):
                return self._recall_view(lookup_target, previous, remaining, on_progress)

        def recover_memory(previous, remaining, on_progress):
            lookup_id, candidates = self.prefetched_memory(previous)
            return recall_prefetched(previous, remaining, on_progress, lookup_id, candidates)

        def recall_prefetched(previous, remaining, on_progress, lookup_id, candidates):
            return self._recall_view(
                lookup_target, previous, remaining, on_progress,
                candidates=candidates, lookup_id=lookup_id,
            )

        def next_view(previous, remaining, on_progress):
            nonlocal next_query_at, sweep
            if self.prefetched_memory is not None:
                lookup_id, candidates = self.prefetched_memory(previous)
                view = recall_prefetched(previous, remaining, on_progress, lookup_id, candidates)
                if view is not None:
                    return view
                if rotation_fallback:
                    if sweep is None:
                        sweep = RotationSweep(
                            self.observe, self.newest_stamp, self.turn, previous, self.cancel,
                            settle_s=self.settle_s,
                        )
                    self._report(on_progress, "exploring", memory_lookup_id=lookup_id)
                    return sweep(previous, remaining, on_progress)
                self._report(on_progress, "memory_fallback_frontier", memory_lookup_id=lookup_id)
                return self(previous, remaining, on_progress)
            can_query = self.query_memory is not None
            queried = can_query and self.checked_views >= next_query_at
            if queried:
                next_query_at = self.checked_views + _MEMORY_QUERY_INTERVAL
                view = recall(previous, remaining, on_progress, "scheduled_lookup")
                if view is not None:
                    return view
            view = self(previous, remaining, on_progress)
            if view is not None:
                return view
            if queried or not can_query:
                if recording:
                    event(logger, "memory_recall_skipped", memory_trigger="frontier_exhausted",
                          reason="already_queried_this_view" if queried else "memory_not_configured",
                          source_stamp_ns=previous.stamp_ns,
                          observation_id=getattr(previous, "observation_id", None))
                return None
            logger.info("No reachable frontier; checking remembered viewpoints")
            next_query_at = self.checked_views + _MEMORY_QUERY_INTERVAL
            return recall(previous, remaining, on_progress, "frontier_exhausted")

        def directed_view(previous, candidate, remaining):
            leg = {"leg_id": self.leg_id + 1, "leg_kind": "directed"}
            view = self.toward_target(previous, candidate, remaining, camera)
            if view is not None:
                self.pending_view = view.stamp_ns, None, leg
                self._report(progress, "checking_exploration_view", **leg)
            return view

        try:
            return search(
                target, budget, progress, next_view=next_view, initial_frame=frame,
                initial_detection=detection, toward_target=directed_view,
                retry_view=retry_view, take_detection=self.take_motion_detection,
                recover_memory=recover_memory if self.prefetched_memory is not None else None,
                on_detection=self.record_detection, exhausted_reason="sweep_complete" if rotation_fallback else "no_reachable_frontier",
            )
        except Exception as exc:
            if not recording:
                raise
            reason = "search_failed"
            if isinstance(exc, TaskCancelled):
                reason = "task_cancelled"
            elif isinstance(exc, TimeoutError):
                reason = "search_timeout"
                try:
                    budget()
                except TimeoutError:
                    reason = "task_timeout"
                except TaskCancelled:
                    reason = "task_cancelled"
            if self.pending_view is not None:
                stamp, _, leg = self.pending_view
                if leg["leg_kind"] == "memory_recall":
                    event(logger, "memory_recall_result", **leg, source_stamp_ns=stamp,
                          phase="redetection", outcome="error", reason=reason,
                          error_type=type(exc).__name__)
            event(
                logger, "search_finished", outcome="cancelled" if reason == "task_cancelled" else "failed",
                reason=reason, error_type=type(exc).__name__, last_leg_id=self.leg_id,
                observed_views=len(self.observed_poses), elapsed_ms=(time.monotonic() - started) * 1000,
            )
            raise
        finally:
            if self.visualization is not None:
                self.visualization.clear()

    def _report(self, on_progress, status, **fields):
        if self.last_frontier_failure is not None:
            fields = {"last_frontier_failure": self.last_frontier_failure, **fields}
        event(logger, "exploration_stage", stage=status, **fields)
        if on_progress is not None:
            on_progress({"status": status, **fields})

    @contextmanager
    def _failed_leg(self, fields, phase):
        try:
            yield
        except Exception as exc:
            if not recording_enabled():
                raise
            reason = {
                TaskCancelled: "task_cancelled",
                TimeoutError: "no_new_view" if phase == "stopped_observation" else "timeout",
            }.get(type(exc), type(exc).__name__)
            if isinstance(exc, ExplorationUnavailable):
                reason = str(exc)
            event(logger, "exploration_leg_finished", **fields, reason=reason,
                  phase=phase, error_type=type(exc).__name__)
            raise

    def _show(self, grid, pose, goal=None):
        if self.visualization is not None:
            self.visualization.update(grid, pose, self.observed_poses, goal=goal)

    def record_detection(self, frame, detection):
        self.checked_views += 1
        pending, self.pending_view = self.pending_view, None
        self.last_frontier_failure = None
        recording = recording_enabled()
        status = detection["status"]
        if recording:
            fields = {
                "observation_id": getattr(frame, "observation_id", None),
                "source_stamp_ns": frame.stamp_ns,
                "detection_status": status,
            }
        goal = None
        if pending is not None and pending[0] == frame.stamp_ns:
            _, goal, leg = pending
            if recording:
                fields.update(leg)
                if leg["leg_kind"] == "memory_recall":
                    outcome = "localized" if status == "localized" else "search_stopped"
                    if status in SEARCH_SEED_STATUSES:
                        outcome = "continue_search"
                    event(logger, "memory_recall_result", **fields, phase="redetection",
                          outcome=outcome,
                          detection_reason=detection.get("detection_error"),
                          target_xyz=detection.get("target_xyz"))
        if status not in _RECORDED_DETECTIONS:
            if recording:
                event(logger, "exploration_view_skipped", **fields, reason="detection_not_usable")
            return
        pose, world_frame = odom_pose(frame.odom)
        if self.world_frame is None:
            self.world_frame = world_frame
        if self.world_frame != world_frame:
            raise ExplorationUnavailable("exploration_frame_changed")
        self.observed_poses.append(pose)
        if goal is not None:
            self.planner.mark_explored(goal)
            if recording:
                fields.update(frontier_xy=goal.frontier_xy, frontier_explored=goal.final)
        if self.latest_map is not None and self.latest_map.stamp_ns == frame.stamp_ns:
            self._show(self.latest_map, pose)
        if recording:
            event(
                logger, "exploration_view_checked", **fields,
                observed_pose=pose, observed_views=len(self.observed_poses),
            )

    def _refresh_view(self, seed, budget):
        if (
            time.monotonic() - seed.received_at > TRACKING_MAX_AGE_S
            or getattr(self.mapper, "last_stamp", 0) > seed.stamp_ns
        ):
            seed = self.observe(
                budget, after_stamp_ns=seed.stamp_ns, max_age_s=TRACKING_MAX_AGE_S,
            )
        pose, frame = odom_pose(seed.odom)
        if self.world_frame is None:
            self.world_frame = frame
        elif self.world_frame != frame:
            raise ExplorationUnavailable("exploration_frame_changed")
        return seed, pose, frame

    def _refresh_map(self, seed, budget):
        seed, _, _ = self._refresh_view(seed, budget)
        grid, pose = self._map(seed)
        self.latest_map = grid
        return seed, grid, pose

    def __call__(self, previous, budget, on_progress):
        def report(status, **fields):
            self._report(on_progress, status, **fields)

        seed = previous
        while True:
            budget()
            seed, grid, pose = self._refresh_map(seed, budget)
            recording = recording_enabled()
            report("selecting_frontier", leg_id=self.leg_id + 1)
            if recording:
                grid_path = diagnostic_artifact(
                    f"frontier-grid-{grid.stamp_ns}.i8", grid.grid.tobytes(),
                    kind="frontier_grid", grid_stamp_ns=grid.stamp_ns, world_frame=grid.frame,
                    shape=grid.grid.shape, dtype=str(grid.grid.dtype),
                    origin_xy=grid.origin_xy, resolution_m=grid.resolution,
                    leg_id=self.leg_id + 1,
                )
                log_context = event_context(grid_stamp_ns=grid.stamp_ns, world_frame=grid.frame,
                                            leg_id=self.leg_id + 1, grid_path=grid_path)
                selection = {
                    "grid_stamp_ns": grid.stamp_ns, "world_frame": grid.frame,
                    "grid_path": grid_path,
                }
            else:
                log_context = nullcontext()
            with log_context:
                goal = self.planner.select(
                    grid.grid, origin_xy=grid.origin_xy, resolution=grid.resolution,
                    robot_xy=pose[:2],
                )
            self._show(grid, pose, goal)
            if goal is None:
                if recording:
                    event(logger, "exploration_frontier_unavailable", **selection)
                return None
            leg = self._begin_leg("frontier", seed)
            fields = {**leg, "frontier_xy": goal.frontier_xy, "goal_xy": goal.xy}
            if recording:
                event(
                    logger, "exploration_goal_selected", **fields, **selection,
                    final=goal.final, motion_budget_s=_VIEW_MOTION_BUDGET_S,
                )
            report("planning_frontier", **fields)
            with self._failed_leg(fields, "motion"):
                result, reason = self._move_to_view(seed, grid.frame, goal.xy, leg, budget, report, fields)

            if result is not None and result["status"] == "arrived":
                return self._capture_view(seed, budget, report, leg, fields, goal=goal)

            self.planner.mark_failed(goal)
            self._show(grid, pose)
            self.last_frontier_failure = {"leg_id": leg["leg_id"], "reason": reason}
            report("frontier_cooldown", **fields, reason=reason)
            event(
                logger, "exploration_leg_finished", **fields,
                reason=reason,
            )
            seed = self.observe(
                budget, after_stamp_ns=seed.stamp_ns, max_age_s=TRACKING_MAX_AGE_S,
            )

    def _capture_view(self, seed, budget, report, leg, fields, *, goal=None):
        report("waiting_exploration_view", **fields)
        with self._failed_leg(fields, "stopped_observation"):
            stopped = observe_stopped(
                self.observe, self.newest_stamp, seed.stamp_ns, budget, self.settle_s,
            )
            stopped, grid, pose = self._refresh_map(stopped, budget)
            self.pending_view = stopped.stamp_ns, goal, leg
            self._show(grid, pose)
        report("checking_exploration_view", **fields)
        event(
            logger, "exploration_leg_finished", **fields, reason="new_view",
            stopped_stamp_ns=stopped.stamp_ns, observation_id=getattr(stopped, "observation_id", None),
        )
        return stopped

    def take_motion_detection(self, frame):
        pending, self.motion_detection = self.motion_detection, None
        if pending is not None and pending[0] is frame:
            return pending[1]
        return None

    def _recall_view(self, target, previous, budget, on_progress, *, candidates=_QUERY_REQUIRED, lookup_id=None):
        def report(status, **fields):
            self._report(on_progress, status, **fields)

        recording = recording_enabled()
        lookup_fields = {"memory_lookup_id": lookup_id or uuid4().hex} if recording else {}
        report("checking_memory" if candidates is _QUERY_REQUIRED else "checking_memory_result", **lookup_fields)
        if candidates is _QUERY_REQUIRED:
            with event_context(**lookup_fields) if recording else nullcontext():
                candidates = self.query_memory(target, budget)
        if not candidates:
            if recording:
                event(logger, "memory_recall_skipped", **lookup_fields, reason="no_candidates")
            return None
        seed, pose, world_frame = self._refresh_view(previous, budget)
        for rank, hit in enumerate(candidates, 1):
            point = (hit.x, hit.y)
            distance = math.dist(point, pose[:2])
            if hit.similarity < _MEMORY_MIN_SIMILARITY:
                reason = "below_similarity"
            elif hit.memory_id in self.recalled_ids:
                reason = "already_recalled"
            else:
                reason = "selected"
            if recording:
                event(
                    logger, "memory_candidate_checked", **lookup_fields,
                    memory_id=hit.memory_id, rank=rank, similarity=hit.similarity,
                    goal_xy=point, goal_yaw=hit.yaw, distance_m=distance,
                    world_frame=world_frame, reason=reason,
                )
            if reason == "selected":
                break
        else:
            if recording:
                event(logger, "memory_recall_skipped", **lookup_fields,
                      reason="no_usable_viewpoint", candidates=len(candidates))
            return None

        self.recalled_ids.add(hit.memory_id)
        leg = {
            **self._begin_leg("memory_recall", seed),
            **lookup_fields, "memory_id": hit.memory_id,
        }
        yaw = hit.yaw
        fields = {**leg, "goal_xy": point, "goal_yaw": yaw, "similarity": hit.similarity}
        if recording:
            event(logger, "memory_goal_selected", **fields, world_frame=world_frame)
        if self.turn is not None and distance >= GOAL_TOLERANCE_M:
            bearing = math.atan2(point[1] - pose[1], point[0] - pose[0])
            if abs(math.remainder(bearing - pose[2], math.tau)) > TURN_TOLERANCE_RAD:
                facing = {**fields, "travel_yaw": bearing}
                report("facing_memory_goal", **facing)
                with self._failed_leg(facing, "turn_before_motion"):
                    turn_reason = self.turn(bearing, world_frame, budget)
                    if turn_reason is not None:
                        event(logger, "exploration_leg_finished", **facing,
                              phase="turn_before_motion", reason=turn_reason)
                        return None
                    def accept_heading(odom):
                        stopped_pose, frame = odom_pose(odom)
                        error = math.remainder(bearing - stopped_pose[2], math.tau)
                        return frame == world_frame and abs(error) <= TURN_TOLERANCE_RAD

                    seed = observe_stopped(
                        self.observe, self.newest_stamp, seed.stamp_ns, budget, self.settle_s,
                        accept_odom=accept_heading,
                    )
                    stopped_pose, _ = odom_pose(seed.odom)
                    event(logger, "memory_goal_heading_ready", **facing,
                          source_stamp_ns=seed.stamp_ns, world_frame=world_frame,
                          heading_error_rad=math.remainder(bearing - stopped_pose[2], math.tau),
                          heading_tolerance_rad=TURN_TOLERANCE_RAD)
        report("planning_memory_goal", **fields)
        with self._failed_leg(fields, "motion"):
            result, reason = self._move_to_view(seed, world_frame, point, leg, budget, report, fields)
        if reason == "target_detected":
            detected_frame, _ = self.motion_detection
            self.pending_view = detected_frame.stamp_ns, None, leg
            event(logger, "exploration_leg_finished", **fields, reason=reason,
                  observation_id=getattr(detected_frame, "observation_id", None))
            return detected_frame
        if result is None or result["status"] != "arrived":
            event(logger, "exploration_leg_finished", **fields, reason=reason)
            return None
        # The point goal carries XY only. Restore the recorded viewing direction before detection.
        if self.turn is not None:
            report("turning_memory_goal", **fields)
            with self._failed_leg(fields, "turn"):
                turn_reason = self.turn(yaw, world_frame, budget)
            if turn_reason is not None:
                event(logger, "memory_heading_unavailable", **fields, reason=turn_reason)
        return self._capture_view(seed, budget, report, leg, fields)

    def _move_to_view(self, seed, world_frame, point, leg, budget, report, fields):
        deadline = time.monotonic() + _VIEW_MOTION_BUDGET_S
        tracking = False
        detection_stopped = threading.Event()
        detection_future = None
        suffix = "memory_goal" if leg.get("leg_kind") == "memory_recall" else "frontier"
        stage = f"planning_{suffix}"

        def leg_budget():
            task_time = budget()
            if detection_future is not None and detection_future.done():
                match = detection_future.result()
                if match is not None:
                    self.motion_detection = match
                    raise _TargetDetected
            left = deadline - time.monotonic()
            if left <= 0:
                raise TimeoutError("Viewpoint movement window ended")
            return min(task_time, left)

        def motion_progress(update):
            nonlocal stage, tracking, detection_future
            if (update["status"] == "tracking" and suffix == "memory_goal"
                    and self.start_motion_detection is not None and detection_future is None):
                detection_future = self.start_motion_detection(seed, detection_stopped, deadline)
            next_stage = {
                "tracking": f"tracking_{suffix}",
                "replanning": f"replanning_{suffix}",
            }.get(update["status"])
            if next_stage is not None and stage != next_stage:
                stage = next_stage
                tracking |= update["status"] == "tracking"
                report(stage, **fields, path_updates=update.get("path_updates"))

        try:
            result = self.approach(
                {
                    "goal_xy": point, "world_frame": world_frame,
                    "observation": {"stamp_ns": seed.stamp_ns}, "timing_ms": {},
                    "exploration_leg": leg,
                },
                leg_budget, self.cancel, motion_progress,
                initial_observation=seed,
                motion_guard=self.cloud.ready if self.cloud is not None else None,
            )
        except _TargetDetected:
            return None, "target_detected"
        except TimeoutError:
            budget()
            return None, "tracking_timeout" if tracking else "no_accepted_path"
        finally:
            detection_stopped.set()
            if detection_future is not None:
                detection_future.cancel()
        return result, result["status"]
