"""Planning and control loops for the path tracker."""

import logging
import math
import threading
import time
from dataclasses import dataclass

import numpy as np

from dimos.plugins.runtime_log import current_task_id, diagnostic_artifact, event, recording_enabled, task_context

from .collision import XNavDPCollisionGuard, collision_veto
from .mpc import solve_mpc
from .plan import MotionPlan

log = logging.getLogger("dimos.path_tracker")
ODOM_MAX_AGE_S = 0.5
GOAL_TOLERANCE_M = 0.2
TURN_TOLERANCE_RAD = math.radians(10)

@dataclass(frozen=True)
class _PathTrace:
    request_stamp_ns: int
    generation: int
    task_id: str | None

    def fields(self):
        return {
            "request_stamp_ns": self.request_stamp_ns,
            "plan_generation": self.generation,
            "task_id": self.task_id,
        }


@dataclass(frozen=True)
class _TurnTrace:
    generation: int
    task_id: str | None
    yaw: float
    world_frame: str

    def fields(self):
        return {
            "request_stamp_ns": None,
            "turn_generation": self.generation,
            "task_id": self.task_id,
            "turn_target_yaw_rad": self.yaw,
            "turn_world_frame": self.world_frame,
        }


def pose_from_odom(odom):
    x, y, yaw = odom
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([x, y, 0.0]), np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class Tracker:
    def __init__(
        self, config, emit, clock=time.monotonic, *, depth_timeout_s=None,
        inputs_ready=None, costmap_timeout_s=None,
    ):
        self.cfg = config["controller"]
        self.use_lidar = bool(config["topics"]["lidar"])
        self.guard = XNavDPCollisionGuard(
            **config["collision"], depth_hard_stop=depth_timeout_s is not None
        )
        self.emit = emit
        self.clock = clock
        self.cv = threading.Condition(threading.RLock())
        self.odom = None
        self.frame = ""
        self.odom_received = 0.0
        self.odom_stamp_ns = None
        self.depth = None
        self.depth_timeout_s = depth_timeout_s
        self.inputs_ready = inputs_ready
        self.motion_guard = None
        self.standoff = None
        self.arrival_reason = None
        self.depth_received = 0.0
        self.depth_stamp_ns = None
        self.depth_valid = False
        self.costmap = None
        self.costmap_timeout_s = costmap_timeout_s
        self.costmap_received = 0.0
        self.costmap_stamp_ns = None
        self._last_costmap_log = None
        self._last_collision_log = None
        self._last_published_veto = None
        self.scan = None
        self.scan_received = 0.0
        self.scan_stamp_ns = None
        self.pending = None
        self.pending_trace = self.active_trace = self.arrival_trace = None
        self.result = None
        self.plan = None
        self.turn_goal = self.turn_trace = None
        self.generation = 0
        self.last_stamp = -1
        self.last_rejected_stamp = None
        self.last_velocity_state = None
        self.running = False
        self.threads = []
        self.stopped = threading.Event()

    def update_odom(self, odom, frame, *, stamp_ns=None):
        with self.cv:
            now = self.clock()
            if self.depth_timeout_s is not None and self.odom is not None and (
                self.frame != frame or now - self.odom_received > ODOM_MAX_AGE_S
            ):
                self._invalidate_plan("odom_discontinuity")
            if self.frame != frame:
                self.costmap = self.guard._latest_costmap = None
                self.costmap_stamp_ns = None
            self.odom, self.frame = odom, frame
            self.odom_received = now
            self.odom_stamp_ns = stamp_ns
            self.cv.notify_all()

    def update_depth(self, depth, *, stamp_ns=None):
        valid = depth is not None and np.any(np.isfinite(depth) & (depth > 0))
        with self.cv:
            now = self.clock()
            if self.depth_timeout_s is not None and (
                not valid
                or (self.depth_valid and now - self.depth_received > self.depth_timeout_s)
            ):
                self._invalidate_plan("depth_invalid" if not valid else "depth_discontinuity")
            self.depth, self.depth_valid = depth, valid
            self.depth_received = now
            self.depth_stamp_ns = stamp_ns
            self.cv.notify_all()

    def set_motion_guard(self, guard):
        with self.cv:
            self.motion_guard = guard

    def _motion_input_error(self):
        if self.depth_timeout_s is not None:
            now = self.clock()
            if self.odom is None or now - self.odom_received > ODOM_MAX_AGE_S:
                return "odom_stale"
            if not self.depth_valid or now - self.depth_received > self.depth_timeout_s:
                return "depth_stale"
        if self.costmap_timeout_s is not None and (
            self.costmap is None or self.clock() - self.costmap_received > self.costmap_timeout_s
        ):
            return "costmap_stale"
        if self.inputs_ready is not None:
            trace = self.active_trace or self.pending_trace or self.turn_trace
            if trace is not None and recording_enabled():
                with task_context(trace.task_id):
                    ready = self.inputs_ready()
            else:
                ready = self.inputs_ready()
            if not ready:
                return "observation_stale"
        if self.motion_guard is not None and not self.motion_guard():
            return "exploration_lidar_stale"
        return None

    def set_standoff(self, target_xy, distance, world_frame):
        with self.cv:
            if self.arrival_reason is not None:
                return False
            self.standoff = (tuple(target_xy), distance, world_frame)
            return True

    def clear_standoff(self):
        with self.cv:
            self.standoff = self.arrival_reason = self.arrival_trace = None

    def _latch_arrival(self, reason):
        if self.standoff is not None and self.arrival_reason is None:
            self.arrival_reason = reason
            target, threshold, frame = self.standoff
            self.arrival_trace = self._invalidate_plan(reason)
            if recording_enabled():
                event(
                    log, "tracker_arrival",
                    **(self.arrival_trace.fields() if self.arrival_trace else {}),
                    reason=reason, world_frame=frame, target_xy=target, odom=self.odom,
                    target_distance_m=math.dist(target, self.odom[:2]),
                    standoff_m=threshold,
                )

    def _standoff_arrival(self):
        if self.standoff is not None and self.arrival_reason is None:
            target_xy, distance, frame = self.standoff
            if (
                self.odom is not None
                and self.frame == frame
                and self.clock() - self.odom_received <= ODOM_MAX_AGE_S
                and math.dist(target_xy, self.odom[:2]) <= distance
            ):
                self._latch_arrival("standoff_reached")
        return self.arrival_reason

    def reached(self, goal_xy, world_frame):
        with self.cv:
            if self._standoff_arrival() is not None:
                return True
            reached = (
                self.odom is not None
                and self.frame == world_frame
                and self.clock() - self.odom_received <= ODOM_MAX_AGE_S
                and math.dist(goal_xy, self.odom[:2]) < GOAL_TOLERANCE_M
            )
            if reached:
                self._latch_arrival("goal_reached")
            return reached

    def _invalidate_plan(self, reason):
        # Call under cv so a running MPC solve cannot commit the old generation.
        active, pending = self.active_trace, self.pending_trace
        turn, self.turn_trace = self.turn_trace, None
        self.generation += 1
        self.pending = self.plan = self.result = self.turn_goal = None
        self.pending_trace = self.active_trace = None
        trace = active or pending
        if trace is not None and recording_enabled():
            event(
                log, "tracker_plan_invalidated", **trace.fields(), reason=reason,
                active_path=active is not None,
                pending_stamp_ns=pending.request_stamp_ns if pending else None,
                new_generation=self.generation,
            )
        return trace or turn

    def clear_plan(self, reason):
        with self.cv:
            self._last_published_veto = None
            stamp = self.result.stamp_ns if self.result is not None else None
            trace = self._invalidate_plan(reason) or self.arrival_trace
            self._emit_command(0.0, 0.0, reason, request_stamp_ns=stamp, trace=trace)
            self.cv.notify_all()

    def turn_to(self, yaw, world_frame, remaining):
        """Wait for the control loop to reach a heading, then publish zero."""
        with self.cv:
            remaining()
            if self.turn_goal is not None:
                return "turn_already_active"
            self._invalidate_plan("exploration_turn_started")
            goal = (yaw, world_frame)
            self.turn_goal = goal
            trace = _TurnTrace(self.generation, current_task_id(), yaw, world_frame) if recording_enabled() else None
            self.turn_trace = trace
            self.cv.notify_all()
            try:
                while self.turn_goal is goal:
                    budget = remaining()
                    _, _, reason = self._turn_command()
                    if reason == "turn_reached":
                        return None
                    if reason != "exploration_turn":
                        return reason
                    self.cv.wait(timeout=min(0.05, budget))
                self._emit_command(0., 0., "exploration_turn_interrupted", trace=trace)
                return "turn_interrupted"
            finally:
                if self.turn_goal is goal:
                    self.clear_plan("exploration_turn_stopped")

    def _turn_command(self, *, collision_details=None):
        yaw, frame = self.turn_goal
        if not self.running:
            return 0., 0., "tracker_stopped"
        reason = self._motion_input_error()
        if reason is not None:
            return 0., 0., reason
        now = self.clock()
        if self.odom is None or now - self.odom_received > ODOM_MAX_AGE_S:
            return 0., 0., "odom_stale"
        if self.frame != frame:
            return 0., 0., "exploration_frame_changed"
        if self.use_lidar and (self.scan is None or now - self.scan_received > 0.75):
            return 0., 0., "exploration_lidar_stale"
        error = math.remainder(yaw - self.odom[2], math.tau)
        if abs(error) <= TURN_TOLERANCE_RAD:
            return 0., 0., "turn_reached"
        speed = self.cfg["max_angular_speed"]
        angular = math.copysign(min(speed, abs(error)), error)
        # The full footprint sweeps a circle as the base turns.
        radius = max(self.guard.robot_radius,
                     math.hypot(self.guard.robot_length / 2, self.guard.robot_half_width))
        scan = self.scan if self.use_lidar else None
        recording = recording_enabled()
        details = (collision_details if collision_details is not None else {}) if recording else None
        if scan is not None:
            near = np.sum(scan[:, :2] ** 2, axis=1) < radius ** 2
            if np.any(near):
                if recording:
                    index = int(near.argmax())
                    details.update(
                        blocking_lidar_index=index, blocking_lidar_base_xy=scan[index, :2].tolist(),
                        footprint_radius_m=radius, footprint_centre_base_xy=(0., 0.),
                    )
                    self._log_collision("lidar", "stop_turn", 0., angular, details)
                return 0., 0., "collision_lidar"
        blocked = collision_veto(
            self.guard, self.odom, self.costmap, scan, self.depth, 0., angular, details=details,
        )
        if blocked is not None:
            if recording:
                self._log_collision(blocked, "stop_turn", 0., angular, details)
            return 0., 0., "collision_" + blocked
        if recording:
            self._last_collision_log = None
        return 0., angular, "exploration_turn"

    def update_costmap(self, costmap, *, stamp_ns=None):
        with self.cv:
            recording = recording_enabled()
            if stamp_ns is not None and self.costmap_stamp_ns is not None:
                if stamp_ns <= self.costmap_stamp_ns:
                    if recording:
                        reason = "duplicate_stamp" if stamp_ns == self.costmap_stamp_ns else "out_of_order_stamp"
                        self._log_costmap("rejected", stamp_ns, reason)
                    return
            self.costmap = self.guard._latest_costmap = costmap
            self.costmap_stamp_ns = stamp_ns
            self.costmap_received = self.clock()
            if recording:
                self._log_costmap("accepted", stamp_ns)
            self.cv.notify_all()

    def _log_costmap(self, outcome, incoming_stamp, reason=None):
        now = self.clock()
        state = (outcome, reason)
        if self._last_costmap_log is not None:
            previous, last = self._last_costmap_log
            if previous == state and now - last < 1.0:
                return
        self._last_costmap_log = state, now
        event(log, f"tracker_costmap_{outcome}", reason=reason,
              incoming_costmap_stamp_ns=incoming_stamp, **self._input_fields())

    def update_scan(self, scan, *, stamp_ns=None):
        with self.cv:
            self.scan = scan
            self.scan_received = self.clock()
            self.scan_stamp_ns = stamp_ns

    def submit(self, result):
        recording = recording_enabled()
        with self.cv:
            if self.turn_goal is not None:
                return False
            if self._standoff_arrival() is not None:
                if recording:
                    event(log, "tracker_path_rejected", request_stamp_ns=result.stamp_ns,
                          reason=self.arrival_reason)
                return False
            input_error = self._motion_input_error()
            if input_error is not None:
                self._invalidate_plan(input_error)
                log.warning(
                    "path rejected request_stamp_ns=%d reason=%s",
                    result.stamp_ns,
                    input_error,
                )
                if recording:
                    event(log, "tracker_path_rejected", request_stamp_ns=result.stamp_ns,
                          reason=input_error, **self._input_fields())
                return False
            # Reject out-of-order observation timestamps.
            if result.stamp_ns <= self.last_stamp:
                if result.stamp_ns != self.last_rejected_stamp:
                    log.warning(
                        "path rejected request_stamp_ns=%d reason=stale last_stamp_ns=%d",
                        result.stamp_ns,
                        self.last_stamp,
                    )
                    self.last_rejected_stamp = result.stamp_ns
                    if recording:
                        event(log, "tracker_path_rejected", request_stamp_ns=result.stamp_ns,
                              reason="stale", last_stamp_ns=self.last_stamp)
                return False
            self.last_stamp = result.stamp_ns
            self.generation += 1
            self.pending = (self.generation, result)
            self.pending_trace = None
            if recording:
                self.pending_trace = _PathTrace(result.stamp_ns, self.generation, current_task_id())
                event(
                    log, "tracker_path_accepted", **self.pending_trace.fields(),
                    world_frame=result.world_frame, goal_xy=tuple(result.goal_world),
                )
            log.info("path accepted request_stamp_ns=%d", result.stamp_ns)
            self.cv.notify_all()
            return True

    def _ready(self):
        return (
            self.odom is not None
            and self.depth_valid
            and self._motion_input_error() is None
        )

    def start(self):
        self.running = True
        self._last_published_veto = None
        self.stopped.clear()
        self.threads = [
            threading.Thread(target=self._planning_loop, name="tracker-mpc"),
            threading.Thread(target=self._control_loop, name="tracker-control"),
        ]
        for thread in self.threads:
            thread.start()
        return self

    def stop(self):
        with self.cv:
            self.running = False
            self.stopped.set()
            trace = self._invalidate_plan("stopped") or self.arrival_trace
            self.cv.notify_all()
        # Control loop checks running while holding the same lock as emission.
        for thread in self.threads:
            thread.join()
        self._emit_command(0.0, 0.0, "stopped", trace=trace)

    def plan_result(self, generation, result, *, trace=None):
        recording = recording_enabled()
        with self.cv:
            if recording and trace is None:
                pending = self.pending_trace
                trace = (
                    pending if pending is not None and pending.generation == generation
                    else _PathTrace(result.stamp_ns, generation, current_task_id())
                )
            if not self._ready() or generation != self.generation:
                if recording:
                    event(log, "tracker_mpc_discarded", **trace.fields(),
                          reason="inputs_or_generation_changed")
                return False
            if result.world_frame != self.frame:
                raise ValueError("result world frame differs from odom")
            current = pose_from_odom(self.odom)
        planning_started = time.perf_counter()
        captured = result.pose
        # Transform the path before solving the MPC problem.
        local_path = (
            result.trajectory[0] @ captured[1].T + captured[0] - current[0]
        ) @ current[1]
        started = self.clock()
        c = self.cfg
        commands = solve_mpc(
            local_path,
            c["mpc_horizon"],
            1.0 / c["tick_rate_hz"],
            c["mpc_desired_v"],
            c["mpc_v_max"],
            min(c["mpc_w_max"], c["max_angular_speed"]),
        )
        world_path = local_path @ current[1].T + current[0]
        plan = MotionPlan(commands, world_path, started, 1.0 / c["tick_rate_hz"])
        elapsed_ms = (time.perf_counter() - planning_started) * 1000
        with self.cv:
            input_error = self._motion_input_error() if generation == self.generation else None
            if input_error is not None:
                self._invalidate_plan(input_error)
            if generation != self.generation or not self.running:
                log.info(
                    "MPC plan discarded request_stamp_ns=%d elapsed_ms=%.1f",
                    result.stamp_ns,
                    elapsed_ms,
                )
                if recording:
                    event(log, "tracker_mpc_discarded", **trace.fields(),
                          reason=input_error or "stale_generation_or_stopped", elapsed_ms=elapsed_ms)
                return False
            self.plan, self.result = plan, result
            self.active_trace = trace
            if self.pending_trace is trace:
                self.pending_trace = None
            log.info(
                "MPC plan ready request_stamp_ns=%d path_points=%d elapsed_ms=%.1f",
                result.stamp_ns,
                len(world_path),
                elapsed_ms,
            )
        if recording:
            event(
                log, "tracker_mpc_ready", **trace.fields(), elapsed_ms=elapsed_ms,
                path_points=len(world_path), first_command=commands[0].tolist() if len(commands) else None,
                plan_started_monotonic_ns=int(plan.started * 1e9), dt_s=plan.dt,
                world_frame=result.world_frame, local_path=local_path.tolist(),
                world_path=world_path.tolist(), commands=commands.tolist(),
            )
        return True

    def preview(self, result):
        """Compute the first command from a frozen input snapshot without emitting it."""
        with self.cv:
            if self.running:
                raise RuntimeError("Use an inactive tracker for one-shot preview")
            self.running = True
            try:
                if not self.plan_result(self.generation, result):
                    raise ValueError("Tracker needs valid depth and odometry")
                linear, angular, reason = self.command()
                return {"linear_x": linear, "angular_z": angular, "reason": reason}
            finally:
                self.plan = self.result = self.active_trace = None
                self.running = False

    def _planning_loop(self):
        while True:
            with self.cv:
                while self.running and (self.pending is None or not self._ready()):
                    self.cv.wait(timeout=0.2)
                if not self.running:
                    return
                generation, result = self.pending
                trace = self.pending_trace
                self.pending = None
            try:
                self.plan_result(generation, result, trace=trace)
            except Exception as exc:
                # Retain the previous plan until its original expiry.
                log.exception(
                    "MPC planning failed request_stamp_ns=%d", result.stamp_ns
                )
                if recording_enabled():
                    fields = trace.fields() if trace else {"request_stamp_ns": result.stamp_ns}
                    event(log, "tracker_mpc_failed", **fields, error_type=type(exc).__name__,
                          reason=str(exc)[:300])

    def command(self, *, collision_details=None):
        """Advance control while holding the condition lock."""
        if not self.running:
            return 0.0, 0.0, "idle"
        if self.turn_goal is not None:
            return self._turn_command(collision_details=collision_details)
        arrival = self._standoff_arrival()
        if arrival is not None:
            return 0.0, 0.0, arrival
        input_error = self._motion_input_error()
        if input_error is not None:
            self._invalidate_plan(input_error)
            return 0.0, 0.0, input_error
        if self.clock() - self.odom_received > ODOM_MAX_AGE_S:
            return 0.0, 0.0, "odom_stale"
        if self.odom is None or not self.depth_valid:
            return 0.0, 0.0, "snapshot_missing"
        if (
            self.result is not None
            and math.dist(self.result.goal_world, self.odom[:2]) < GOAL_TOLERANCE_M
        ):
            self._latch_arrival("goal_reached")
            self.plan = None
            return 0.0, 0.0, "goal_reached"
        plan = self.plan
        if plan is None:
            return 0.0, 0.0, "no_plan"
        c = self.cfg
        now = self.clock()
        v, w = plan.command(now, c["decel_start_s"], c["stop_s"] - c["decel_start_s"])
        if (
            now - plan.started
            >= min(c["decel_start_s"], len(plan.commands) * plan.dt)
            + c["stop_s"]
            - c["decel_start_s"]
        ):
            return 0.0, 0.0, "expired"
        scan = self.scan if self.use_lidar else None
        if self.depth_timeout_s is not None and now - self.scan_received > 0.75:
            scan = None
        recording = recording_enabled()
        details = (collision_details if collision_details is not None else {}) if recording else None
        blocked = collision_veto(
            self.guard, self.odom, self.costmap, scan, self.depth, v, w, details=details,
        )
        rotate_only = False
        rotation_details = None
        rotation_blocked = None
        if blocked is not None and abs(w) > 1e-3:
            rotation_details = {} if recording else None
            rotation_blocked = collision_veto(
                self.guard, self.odom, self.costmap, scan, self.depth, 0.0, w,
                details=rotation_details,
            )
            rotate_only = rotation_blocked is None
        if blocked is not None and recording:
            self._log_collision(
                blocked, "rotate_only" if rotate_only else "stop", v, w, details,
                rotation_blocked=rotation_blocked, rotation_details=rotation_details,
            )
            if collision_details is not None and rotation_details is not None:
                collision_details["rotation_check"] = rotation_details
        elif blocked is None and recording:
            self._last_collision_log = None
        if blocked is not None and not rotate_only:
            v, w, reason = 0.0, 0.0, "collision_" + blocked
        elif rotate_only:
            v, reason = 0.0, "rotate_only"
        else:
            reason = "tracking"
        return (
            float(v),
            float(np.clip(w, -c["max_angular_speed"], c["max_angular_speed"])),
            reason,
        )

    def _log_collision(self, reason, action, v, w, details, *, rotation_blocked=None, rotation_details=None):
        trace = self.active_trace or self.turn_trace
        state = (trace, reason, action, rotation_blocked)
        now = self.clock()
        if self._last_collision_log is not None:
            previous, last = self._last_collision_log
            if previous == state and now - last < 1.0:
                return
        self._last_collision_log = state, now
        trace_fields = trace.fields() if trace else {
            "request_stamp_ns": self.result.stamp_ns if self.result is not None else None,
        }
        grid_path = None
        if reason in {"map", "map_bounds"} or rotation_blocked in {"map", "map_bounds"}:
            grid = self.costmap if self.costmap is not None else self.guard._latest_costmap
            if grid is not None:
                grid_path = diagnostic_artifact(
                    f"collision-grid-{self.costmap_stamp_ns}.i8", grid.grid.tobytes(),
                    **trace_fields, kind="collision_costmap", shape=grid.grid.shape,
                    dtype=str(grid.grid.dtype), resolution_m=grid.resolution,
                    origin_xy=(grid.origin.position.x, grid.origin.position.y), world_frame=self.frame,
                    costmap_stamp_ns=self.costmap_stamp_ns,
                )
        event(
            log, "tracker_collision_veto", **trace_fields, **self._input_fields(), **details,
            reason=reason, action=action, proposed_linear_x=float(v) if math.isfinite(v) else None,
            proposed_angular_z=float(w) if math.isfinite(w) else None, costmap_path=grid_path,
            rotation_blocked=rotation_blocked, rotation_details=rotation_details,
        )

    def _input_fields(self):
        now = self.clock()
        return {
            "odom": self.odom,
            "world_frame": self.frame,
            "odom_stamp_ns": self.odom_stamp_ns,
            "depth_stamp_ns": self.depth_stamp_ns,
            "lidar_stamp_ns": self.scan_stamp_ns,
            "costmap_stamp_ns": self.costmap_stamp_ns,
            "costmap_timeout_s": self.costmap_timeout_s,
            "costmap_age_ms": (now - self.costmap_received) * 1000 if self.costmap is not None else None,
            "odom_age_ms": (now - self.odom_received) * 1000 if self.odom is not None else None,
            "depth_age_ms": (now - self.depth_received) * 1000 if self.depth is not None else None,
            "lidar_age_ms": (now - self.scan_received) * 1000 if self.scan is not None else None,
        }

    def _emit_command(self, v, w, reason, *, request_stamp_ns=None, trace=None, collision_details=None):
        trace = trace or self.turn_trace or self.active_trace or self.arrival_trace
        stamp = (
            getattr(trace, "request_stamp_ns", None) if trace is not None
            else self.result.stamp_ns if self.result is not None else request_stamp_ns
        )
        recording = recording_enabled()
        dispatch_ns = time.monotonic_ns() if recording else None
        fields = (trace.fields() if trace else {"request_stamp_ns": stamp}) if recording else None
        try:
            published = self.emit(v, w, reason)
        except Exception as exc:
            if recording:
                event(
                    log, "tracker_command_failed", **fields, reason=reason,
                    dispatch_monotonic_ns=dispatch_ns, error_type=type(exc).__name__,
                    linear_x=v, angular_z=w,
                )
            raise
        if type(published) is not bool:
            published = None
        if recording:
            costmap_stamp_ns = self.costmap_stamp_ns
            check = {"collision_check": collision_details} if collision_details else {}
            event(
                log, "tracker_command_sample", **fields, reason=reason, published=published,
                dispatch_monotonic_ns=dispatch_ns, issued_monotonic_ns=time.monotonic_ns(),
                linear_x=v, angular_z=w, costmap_stamp_ns=costmap_stamp_ns, **check,
            )
            previous = self._last_published_veto
            if published and (reason.startswith("collision_") or reason == "rotate_only"):
                self._last_published_veto = (fields, reason)
            elif published and (v or w) and reason in {"tracking", "exploration_turn"} and previous:
                self._last_published_veto = None
                previous_fields, previous_reason = previous
                if (
                    previous_fields.get("task_id") == fields.get("task_id")
                    and previous_fields.get("turn_generation") == fields.get("turn_generation")
                ):
                    event(
                        log, "tracker_collision_recovered", **fields, reason=reason,
                        previous_reason=previous_reason,
                        previous_request_stamp_ns=previous_fields.get("request_stamp_ns"),
                        linear_x=v, angular_z=w, costmap_stamp_ns=costmap_stamp_ns, **check,
                    )
        state = (stamp, reason, published, bool(v or w))
        if state != self.last_velocity_state:
            self.last_velocity_state = state
            log.info(
                "velocity state changed request_stamp_ns=%s reason=%s published=%s linear_x=%.3f angular_z=%.3f",
                stamp, reason, published, v, w,
            )
            if recording:
                event(
                    log, "tracker_velocity", **fields, **self._input_fields(),
                    reason=reason, published=published, linear_x=v, angular_z=w,
                )

    def _control_loop(self):
        period = 1.0 / self.cfg["tick_rate_hz"]
        while True:
            started = self.clock()
            with self.cv:
                if not self.running:
                    return
                trace = self.turn_trace or self.active_trace or self.arrival_trace
                try:
                    details = {} if recording_enabled() else None
                    self._emit_command(
                        *self.command(collision_details=details), trace=trace, collision_details=details,
                    )
                except Exception:
                    log.exception(
                        "control tick failed request_stamp_ns=%s",
                        self.result.stamp_ns if self.result is not None else None,
                    )
                    self._emit_command(0.0, 0.0, "tick_error", trace=trace)
            self.stopped.wait(timeout=max(0.0, period - (self.clock() - started)))
