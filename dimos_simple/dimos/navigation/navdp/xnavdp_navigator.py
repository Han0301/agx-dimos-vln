# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Opt-in point-goal navigator. The legacy NavDP implementation is untouched."""

from __future__ import annotations

import math
import threading
import time
from typing import Any, ClassVar, NamedTuple

import numpy as np

from dimos.core.core import rpc
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.navigation.base import NavigationState
from dimos.navigation.navdp import navigator as legacy
from dimos.navigation.navdp.pointgoal import XNavDPClient, local_goal, path_lookahead
from dimos.navigation.navdp.xnavdp_tracking import MotionPlan, collision_veto, solve_mpc
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class PointSnapshot(NamedTuple):
    rgb: tuple[np.ndarray, float]
    depth: tuple[np.ndarray, float]
    captured_pose: tuple[np.ndarray, np.ndarray]
    current_pose: tuple[np.ndarray, np.ndarray]
    target: tuple[float, float]
    epoch: int


def _pose_from_xyyaw(
    odom: tuple[float, float, float] | None,
) -> tuple[np.ndarray, np.ndarray] | None:
    if odom is None:
        return None
    x, y, yaw = odom
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return (
        np.array([x, y, 0.0]),
        np.array(
            [
                [cosine, -sine, 0.0],
                [sine, cosine, 0.0],
                [0.0, 0.0, 1.0],
            ]
        ),
    )


class XNavDPNavigator(legacy.NavDPNavigator):
    """Use a distinct class name; inherited NavDPNavigator RPC aliases remain available."""

    dedicated_worker: ClassVar[bool] = True

    def __init__(
        self,
        xnavdp_max_angular_speed: float = 0.2,
        xnavdp_rate_hz: float = 5.0,
        xnavdp_lookahead_m: float = 1.5,
        xnavdp_decel_start_s: float = 1.0,
        xnavdp_stop_s: float = 3.0,
        **kwargs: Any,
    ) -> None:
        # Official XNavDP trajectories are base-local, not camera-local.
        kwargs.update(
            cam_x=0.0, cam_y=0.0, cam_z=0.0, cam_pitch=0.0, enable_internal_vlm=False
        )
        super().__init__(**kwargs)
        if not 0 < xnavdp_rate_hz <= 20:
            raise ValueError("Invalid XNavDP timing")
        self._xclient = XNavDPClient(self._navdp_url)
        self._xlock = threading.Lock()
        self._xrgb: tuple[np.ndarray, float] | None = None
        self._xdepth: tuple[np.ndarray, float] | None = None
        self._xframe = ""
        self._xepoch = 0
        if not math.isfinite(xnavdp_max_angular_speed) or xnavdp_max_angular_speed <= 0:
            raise ValueError("xnavdp_max_angular_speed must be finite and positive")
        self._xmax_angular_speed = xnavdp_max_angular_speed
        self._xperiod = 1 / xnavdp_rate_hz
        self._xlookahead = xnavdp_lookahead_m
        self._xlast_rgb = -1.0
        self._xlast_error = 0.0
        self._xlast_guard_log = 0.0
        self._xplan: MotionPlan | None = None
        self._xodom_received = 0.0
        self._xcontrol_log = 0.0
        if not (math.isfinite(xnavdp_decel_start_s) and math.isfinite(xnavdp_stop_s)
                and 0 < xnavdp_decel_start_s < xnavdp_stop_s):
            raise ValueError("XNavDP requires 0 < decel_start_s < stop_s")
        self._xdecel_start_s = xnavdp_decel_start_s
        self._xstop_s = xnavdp_stop_s
        self._xstats_lock = threading.Lock()
        self._xstage_stats: dict[str, list[float]] = {}
        self._xoutcomes: dict[str, int] = {}
        self._xstats_started = time.monotonic()

    @rpc
    def start(self) -> None:
        # Verify BEFORE legacy start() can POST navigator_reset to the endpoint.
        self._xclient.verify_backend()
        super().start()

    def _on_navdp_image(self, img: Any) -> None:
        super()._on_navdp_image(img)
        with self._xlock:
            self._xrgb = (legacy._image_to_bgr(img), float(img.ts))

    def _on_navdp_depth(self, img: Any) -> None:
        super()._on_navdp_depth(img)
        with self._xlock:
            self._xdepth = (img.data, float(img.ts))

    def _on_odom(self, odom: PoseStamped) -> None:
        super()._on_odom(odom)
        self._xframe = odom.frame_id
        self._xodom_received = time.monotonic()

    def _snapshot(self) -> PointSnapshot | None:
        with self._xlock:
            rgb, depth = self._xrgb, self._xdepth
            epoch = self._xepoch
        if rgb is None or depth is None:
            return None
        if rgb[0].shape[:2] != depth[0].shape:
            return None
        if not np.any(np.isfinite(depth[0]) & (depth[0] > 0)):
            return None
        with self._lock:
            final = self._goal_pose
            path = list(self._guidance_path or [])
            odom = self._latest_odom
        pose = _pose_from_xyyaw(odom)
        if pose is None:
            return None
        if final is not None:
            target = (final.position.x, final.position.y)
        elif path:
            target = path_lookahead(path, (odom[0], odom[1]), self._xlookahead)
        else:
            return None
        if final is None and math.dist(target, pose[0][:2]) < 0.2:
            return None
        return PointSnapshot(rgb, depth, pose, pose, target, epoch)

    @rpc
    def set_point_goal(self, goal: PoseStamped, valid_for_s: float = 2.0) -> bool:
        del valid_for_s
        if not self._xframe or goal.frame_id != self._xframe:
            return False
        if not np.all(np.isfinite([goal.position.x, goal.position.y])):
            return False
        first = self._goal_pose is None
        accepted = super().set_goal(goal)
        with self._xlock:
            if first:
                self._xepoch += 1
                self._xplan = None
        if self._state_machine is not None:
            self._state_machine.state = legacy._navdp_bridge["NavState"].APPROACH
        return accepted

    @rpc
    def set_goal(self, goal: PoseStamped) -> bool:
        if not self._xframe or goal.frame_id != self._xframe:
            return False
        accepted = super().set_goal(goal)
        with self._xlock:
            self._xepoch += 1
            self._xplan = None
        return accepted

    @rpc
    def set_language_goal(self, goal: str) -> bool:
        if goal.strip().lower() != "explore the environment":
            logger.warning(
                "XNavDP needs a coordinate goal; text is handled by VLN skills"
            )
            return False
        with self._xlock:
            self._xepoch += 1
            self._xplan = None
        with self._lock:
            self._goal_pose = None
        return super().set_language_goal(goal)

    @rpc
    def set_reference_image(self, image_bgr: np.ndarray) -> bool:
        logger.warning("XNavDP supports point goals only; use set_point_goal")
        return False

    @rpc
    def cancel_goal(self) -> bool:
        with self._xlock:
            self._xepoch += 1
            self._xplan = None
        with self._infer_lock:
            self._latest_traj = None
            self._traj_timestamp = 0
        return super().cancel_goal()

    @rpc
    def pause_motion(self) -> bool:
        # Invalidate in-flight inference as well as the buffered action sequence.
        with self._xlock:
            self._xepoch += 1
            self._xplan = None
            self._motion_paused = True
        return super().pause_motion()

    def _inference_loop(self) -> None:
        verified = False
        try:
            while self._running:
                started = time.monotonic()
                snapshot = self._snapshot()
                if (
                    self._nav_state == NavigationState.IDLE
                    or self._motion_paused
                    or snapshot is None
                ):
                    time.sleep(0.05)
                    continue
                rgb, depth, pose, _, target, epoch = snapshot
                if rgb[1] <= self._xlast_rgb:
                    time.sleep(0.01)
                    continue
                try:
                    if not verified:
                        self._xclient.verify_backend()
                        if (
                            self._navdp_client is None
                            or not self._navdp_client.initialized
                        ):
                            raise RuntimeError("XNavDP server reset did not succeed")
                        verified = True
                    point = local_goal(target, *pose)
                    q = Quaternion.from_rotation_matrix(pose[1])
                    quaternion = np.array([q.x, q.y, q.z, q.w])
                    http_started = time.monotonic()
                    try:
                        traj, all_traj, values = self._xclient.pointgoal_step(
                            rgb[0], depth[0], point, pose[0], quaternion
                        )
                    finally:
                        self._x_record_duration("http", time.monotonic() - http_started)
                    current = _pose_from_xyyaw(self._latest_odom)
                    latest = self._snapshot()
                    if (
                        current is None
                        or latest is None
                        or latest.epoch != epoch
                        or not self._running
                    ):
                        continue
                    if self._nav_state == NavigationState.IDLE:
                        continue

                    # Rebase delayed predictions into the robot's current frame.
                    def rebase(
                        a: np.ndarray,
                        capture_pose: tuple[np.ndarray, np.ndarray] = pose,
                        current_pose: tuple[np.ndarray, np.ndarray] = current,
                    ) -> np.ndarray:
                        return (
                            a @ capture_pose[1].T
                            + capture_pose[0]
                            - current_pose[0]
                        ) @ current_pose[1]

                    # Planning/optimization stays off the periodic control thread.
                    local_path = rebase(traj[0])
                    plan_started = time.monotonic()
                    try:
                        commands = solve_mpc(
                            local_path, self._mpc_horizon, 1.0 / self._tick_rate_hz,
                            self._mpc_desired_v, self._mpc_v_max,
                            min(self._mpc_w_max, self._xmax_angular_speed),
                        )
                    finally:
                        self._x_record_duration("mpc", time.monotonic() - plan_started)
                    world_path = local_path @ current[1].T + current[0]
                    plan = MotionPlan(commands, world_path, plan_started, 1.0 / self._tick_rate_hz)
                    with self._xlock:
                        if (self._xepoch != epoch or self._motion_paused
                                or self._nav_state == NavigationState.IDLE):
                            continue
                        with self._infer_lock:
                            self._xplan = plan
                            self._latest_traj = local_path
                            self._latest_all_traj = rebase(all_traj)
                            self._latest_all_vals = values
                            self._traj_timestamp = time.time()
                            self._infer_mode = "pointgoal"
                    self._xlast_rgb = rgb[1]
                except Exception as exc:
                    self._x_count("inference_error")
                    if time.monotonic() - self._xlast_error > 5:
                        self._xlast_error = time.monotonic()
                        logger.warning("XNavDP pointgoal failed: %s", exc)
                    time.sleep(0.5)
                time.sleep(max(0, self._xperiod - (time.monotonic() - started)))
        finally:
            self._xclient.close()

    def _tick_guard_checks(self) -> Any:
        # IDLE relinquishes velocity ownership to the exclusive turn/A* controller.
        # cancel_goal already publishes the final stop once.
        if self._nav_state == NavigationState.IDLE:
            self._x_count("idle")
            return None
        snapshot = self._snapshot()
        if snapshot is None:
            if time.monotonic() - self._xlast_guard_log >= 5:
                logger.warning(
                    "[XNavDP-Wait] No control snapshot: rgb=%s depth=%s odom=%s path_points=%d point_goal=%s",
                    self._xrgb is not None,
                    self._xdepth is not None,
                    self._latest_odom is not None,
                    len(self._guidance_path or []),
                    self._goal_pose is not None,
                )
                self._xlast_guard_log = time.monotonic()
            self._x_stop("snapshot_missing")
            return None
        if self._goal_pose is not None:
            target, position = snapshot.target, snapshot.current_pose[0]
            if math.dist(target, position[:2]) < 0.2:
                self.cancel_goal()
                self._goal_reached = True
                self._x_stop("goal_reached")
                return None
        # Retain pause/IDLE guards while using D455 for the control snapshot.
        if (self._motion_paused or self._nav_state == NavigationState.IDLE
                or time.monotonic() - self._xodom_received > 0.5):
            with self._xlock:
                self._xplan = None
            self._x_stop("paused" if self._motion_paused else "odom_stale")
            return None
        with self._lock:
            odom = self._latest_odom
        self._update_explore_trail(odom[0], odom[1])
        return self._TickContext(image=snapshot.rgb[0], odom=odom)

    def _tick_state_machine_update(self, ctx: Any) -> bool:
        # Arrival is based on the coordinate goal, never centre-image depth.
        with self._xlock:
            ctx.sm_depth = (
                self._xdepth[0] if self._xdepth is not None else np.zeros((1, 1))
            )
        return False

    def _control_loop(self) -> None:
        """Periodic command output; HTTP and MPC never run on this thread."""
        period = 1.0 / self._tick_rate_hz
        self._xstats_started = time.monotonic()
        while self._running:
            started = time.monotonic()
            try:
                self._tick()
            except Exception:
                self._x_stop("tick_error")
                logger.exception("XNavDP control tick failed; stopping")
            self._x_record_duration("tick", time.monotonic() - started)
            self._x_report_stats()
            time.sleep(max(0.0, period - (time.monotonic() - started)))

    def _tick(self) -> None:
        ctx = self._tick_guard_checks()
        if ctx is None:
            return
        self._tick_state_machine_update(ctx)
        with self._xlock:
            plan = self._xplan
        if plan is None:
            self._x_stop("no_plan")
            return
        now = time.monotonic()
        valid_s = self._xdecel_start_s
        decel_s = self._xstop_s - valid_s
        ctx.v, ctx.w = plan.command(now, valid_s, decel_s)
        if now - plan.started >= min(valid_s, len(plan.commands) * plan.dt) + decel_s:
            self._x_stop("expired")
            return
        pose = _pose_from_xyyaw(ctx.odom)
        ctx.traj = (plan.world_path - pose[0]) @ pose[1]
        ctx.waypoints = ctx.traj[:, :2]
        with self._lock:
            costmap, scan = self._latest_costmap, self._latest_scan_points
        collision_started = time.monotonic()
        try:
            blocked_reason = collision_veto(
                self._trajectory_selector, ctx.odom, costmap, scan, ctx.sm_depth,
                ctx.v, ctx.w,
            )
            rotate_only = False
            if blocked_reason is not None and abs(ctx.w) > 1e-3:
                rotate_reason = collision_veto(
                    self._trajectory_selector, ctx.odom, costmap, scan,
                    ctx.sm_depth, 0.0, ctx.w,
                )
                rotate_only = rotate_reason is None
            else:
                rotate_reason = None
        finally:
            self._x_record_duration("collision", time.monotonic() - collision_started)
        blocked = blocked_reason is not None and not rotate_only
        self._last_sel_fallback = blocked
        self._last_sel_index = -1
        self._last_sel_cost = 0.0
        if blocked:
            ctx.v = ctx.w = 0.0
        elif rotate_only:
            ctx.v = 0.0
        # Recheck cancellation/plan replacement after geometric checks.
        with self._xlock:
            if self._xplan is not plan or self._motion_paused or self._nav_state == NavigationState.IDLE:
                self._x_count("superseded")
                return
            # Collision checking can be slow: re-evaluate expiry at send time.
            send_age = time.monotonic() - plan.started
            if blocked_reason is None:
                ctx.v, ctx.w = plan.command(time.monotonic(), valid_s, decel_s)
            elif rotate_only:
                _, ctx.w = plan.command(time.monotonic(), valid_s, decel_s)
                ctx.v = 0.0
            if blocked:
                reason = "collision_" + blocked_reason
                if rotate_reason is not None:
                    self._x_count("rotation_blocked_" + rotate_reason)
            elif rotate_only:
                reason = "rotate_only"
            elif send_age >= min(valid_s, len(plan.commands) * plan.dt) + decel_s:
                reason = "expired"
            elif abs(ctx.v) < 1e-6 and abs(ctx.w) < 1e-6:
                reason = "policy_zero"
            else:
                reason = "decelerating" if send_age >= valid_s else "tracking"
            self._x_count(reason)
            if time.monotonic() - self._xcontrol_log >= 5.0:
                self._xcontrol_log = time.monotonic()
                logger.info("XNavDP execution: selected=policy age=%.3fs blocked=%s v=%.3f w=%.3f",
                            send_age, blocked, ctx.v, ctx.w)
            publish_started = time.monotonic()
            try:
                self._tick_publish(ctx)
            finally:
                self._x_record_duration("publish", time.monotonic() - publish_started)

    def _x_count(self, reason: str) -> None:
        with self._xstats_lock:
            self._xoutcomes[reason] = self._xoutcomes.get(reason, 0) + 1

    def _x_record_duration(self, stage: str, duration_s: float) -> None:
        with self._xstats_lock:
            stat = self._xstage_stats.setdefault(stage, [0.0, 0.0, 0.0])
            stat[0] += 1
            stat[1] += duration_s
            stat[2] = max(stat[2], duration_s)

    def _x_stop(self, reason: str) -> None:
        self._x_count(reason)
        started = time.monotonic()
        try:
            self.cmd_vel.publish(Twist())
        finally:
            self._x_record_duration("publish", time.monotonic() - started)

    def _x_report_stats(self) -> None:
        now = time.monotonic()
        if now - self._xstats_started < 5.0:
            return
        with self._xstats_lock:
            elapsed = now - self._xstats_started
            stages, outcomes = self._xstage_stats, self._xoutcomes
            self._xstage_stats, self._xoutcomes = {}, {}
            self._xstats_started = now
        timings = {name: {"n": int(v[0]), "mean_ms": round(1000 * v[1] / v[0], 2),
                          "max_ms": round(1000 * v[2], 2)} for name, v in stages.items()}
        logger.info("[XNavDP-Perf] window_s=%.2f tick_hz=%.2f stages=%s outcomes=%s",
                    elapsed, stages.get("tick", [0])[0] / elapsed, timings, outcomes)

    def _tick_publish(self, ctx: Any) -> None:
        ctx.w = float(np.clip(ctx.w, -self._xmax_angular_speed, self._xmax_angular_speed))
        super()._tick_publish(ctx)

    def _tick_escape_override(self, ctx: Any) -> bool:
        # XNavDP owns recovery through its selected forward/reverse trajectory.
        return False


# Preserve the import API without duplicating names in the class inheritance chain.
NavDPNavigator = XNavDPNavigator
xnavdp_navigator = XNavDPNavigator.blueprint
