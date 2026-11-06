"""Standalone XNavDP navigator with no NavDP or navdp_bridge dependency."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
import threading
import time
from typing import Any

import numpy as np
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.navigation.base import NavigationInterface, NavigationState
from dimos.navigation.xnavdp.client import XNavDPClient
from dimos.navigation.xnavdp.pointgoal import local_goal, path_lookahead
from dimos.navigation.xnavdp.tracking import (
    MotionPlan,
    XNavDPCollisionGuard,
    collision_veto,
    solve_mpc,
)
from dimos.utils.logging_config import setup_logger
from dimos_lcm.std_msgs import Bool

logger = setup_logger()


@dataclass
class _TickContext:
    image: np.ndarray
    odom: tuple[float, float, float]
    depth: np.ndarray
    traj: np.ndarray | None = None
    waypoints: np.ndarray | None = None
    v: float = 0.0
    w: float = 0.0


@dataclass(frozen=True)
class _Snapshot:
    rgb: tuple[np.ndarray, float]
    depth: tuple[np.ndarray, float]
    pose: tuple[np.ndarray, np.ndarray]
    target: tuple[float, float]
    epoch: int


class NavDPNavigator(NavigationInterface):
    """Compatibility interface name used by existing DimOS RPC bindings."""


def _pose_to_xyyaw(pose: PoseStamped) -> tuple[float, float, float]:
    q = pose.orientation
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return pose.position.x, pose.position.y, math.atan2(siny, cosy)


def _pose_from_xyyaw(
    odom: tuple[float, float, float] | None,
) -> tuple[np.ndarray, np.ndarray] | None:
    if odom is None:
        return None
    x, y, yaw = odom
    c, s = math.cos(yaw), math.sin(yaw)
    return (
        np.array([x, y, 0.0]),
        np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]),
    )


def _image_to_bgr(img: Image) -> np.ndarray:
    data = img.data
    if img.format == ImageFormat.RGB:
        import cv2

        data = cv2.cvtColor(data, cv2.COLOR_RGB2BGR)
    return data


class _RosDebugPublisher:
    def __init__(self, enabled: bool) -> None:
        self.node = None
        self.context = None
        self.result_pub = None
        self.path_pub = None
        self._error_logged = False
        if enabled:
            try:
                import rclpy
                from rclpy.context import Context
                from rclpy.node import Node
                from nav_msgs.msg import Path as ROSPath
                from std_msgs.msg import String

                self.context = Context()
                rclpy.init(context=self.context)
                self.node = Node(f"xnavdp_debug_{os.getpid()}", context=self.context)
                self._path_type = ROSPath
                self.result_pub = self.node.create_publisher(String, "/xnavdp/result", 10)
                self.path_pub = self.node.create_publisher(ROSPath, "/xnavdp/trajectory", 10)
            except Exception:
                logger.warning("XNavDP ROS debug publishers unavailable", exc_info=True)
                self.close()

    def publish(
        self,
        trajectory: np.ndarray,
        candidates: np.ndarray,
        values: np.ndarray,
        goal: np.ndarray,
        position: np.ndarray,
        quaternion: np.ndarray,
        http_seconds: float,
        trace_id: str,
    ) -> None:
        if self.node is None or self.result_pub is None:
            return
        try:
            from geometry_msgs.msg import PoseStamped as ROSPoseStamped
            from std_msgs.msg import String

            payload = {
                "trajectory": trajectory.tolist(),
                "all_trajectory": candidates.tolist(),
                "all_values": values.reshape(1, -1).tolist(),
                "trajectory_frame": "base_link",
                "goal_frame": "base_link",
                "goal": goal.tolist(),
                "robot_pos": position.tolist(),
                "robot_quat": quaternion.tolist(),
                "http_seconds": http_seconds,
                "roundtrip_seconds": http_seconds,
                "transport": "ros2",
                "request_id": trace_id,
            }
            self.result_pub.publish(String(data=json.dumps(payload, allow_nan=False)))
            path = self._path_type()
            path.header.frame_id = "base_link"
            path.header.stamp = self.node.get_clock().now().to_msg()
            for x, y, z in trajectory[0]:
                pose = ROSPoseStamped()
                pose.header = path.header
                pose.pose.position.x = float(x)
                pose.pose.position.y = float(y)
                pose.pose.position.z = float(z)
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
            self.path_pub.publish(path)
        except Exception:
            if not self._error_logged:
                self._error_logged = True
                logger.warning("XNavDP debug publish failed", exc_info=True)

    def close(self) -> None:
        if self.node is not None:
            try:
                self.node.destroy_node()
            except Exception:
                pass
            self.node = None
        if self.context is not None:
            try:
                import rclpy

                if self.context.ok():
                    rclpy.shutdown(context=self.context)
            except Exception:
                pass
            self.context = None


class XNavDPNavigator(Module, NavDPNavigator):
    """XNavDP point-goal execution with a dedicated HTTP and MPC pipeline."""

    dedicated_worker = True

    color_image: In[Image]
    navdp_image: In[Image]
    depth_image: In[Image]
    navdp_depth: In[Image]
    odom: In[PoseStamped]
    global_costmap: In[OccupancyGrid]
    explore_cmd: In[Bool]
    stop_explore_cmd: In[Bool]
    realsense_camera_info: In[CameraInfo]
    cmd_vel: Out[Twist]
    navdp_path: Out[Path]

    def __init__(
        self,
        navdp_server_url: str = "ros2:///xnavdp",
        *,
        tick_rate_hz: float = 10.0,
        cam_intrinsic: np.ndarray | None = None,
        vlm_server_url: str = "",
        cam_x: float = 0.0,
        cam_y: float = 0.0,
        cam_z: float = 0.0,
        cam_pitch: float = 0.0,
        mpc_horizon: int = 15,
        mpc_desired_v: float = 0.4,
        mpc_v_max: float = 0.4,
        mpc_w_max: float = 0.2,
        mpc_ref_gap: int = 3,
        goal_lookahead_m: float = 1.5,
        hold_duration_s: float = 0.0,
        decel_duration_s: float = 0.5,
        enable_internal_vlm: bool = False,
        goal_image_dir: str = "",
        trajectory_selector_enabled: bool = True,
        trajectory_selector_kwargs: dict[str, Any] | None = None,
        xnavdp_rate_hz: float = 5.0,
        xnavdp_lookahead_m: float = 1.5,
        xnavdp_max_angular_speed: float = 0.2,
        xnavdp_decel_start_s: float = 1.0,
        xnavdp_stop_s: float = 3.0,
        debug_mode: bool = True,
        **kwargs: Any,
    ) -> None:
        self._navdp_url = navdp_server_url
        self._tick_rate_hz = float(tick_rate_hz)
        self._cam_intrinsic = (
            np.asarray(cam_intrinsic, dtype=float) if cam_intrinsic is not None else None
        )
        self._intrinsic = self._cam_intrinsic
        self._mpc_horizon = int(mpc_horizon)
        self._mpc_desired_v = float(mpc_desired_v)
        self._mpc_v_max = float(mpc_v_max)
        self._mpc_w_max = float(mpc_w_max)
        self._xmax_angular_speed = float(xnavdp_max_angular_speed)
        if not 0 < xnavdp_rate_hz <= 20:
            raise ValueError("Invalid XNavDP timing")
        if not (0 < xnavdp_decel_start_s < xnavdp_stop_s):
            raise ValueError("XNavDP requires 0 < decel_start_s < stop_s")
        self._xperiod = 1.0 / float(xnavdp_rate_hz)
        self._xlookahead = float(xnavdp_lookahead_m)
        self._xdecel_start_s = float(xnavdp_decel_start_s)
        self._xstop_s = float(xnavdp_stop_s)
        self.debug_mode = bool(debug_mode)
        del (
            vlm_server_url,
            cam_x,
            cam_y,
            cam_z,
            cam_pitch,
            mpc_ref_gap,
            goal_lookahead_m,
            hold_duration_s,
            decel_duration_s,
            enable_internal_vlm,
            goal_image_dir,
        )

        selector = dict(trajectory_selector_kwargs or {})
        selector.setdefault("enabled", trajectory_selector_enabled)
        self._collision_guard = XNavDPCollisionGuard(**selector)

        self._xclient = XNavDPClient(self._navdp_url, debug_mode=self.debug_mode)
        self._lock = threading.Lock()
        self._xlock = threading.Lock()
        self._infer_lock = threading.Lock()
        self._plan_cv = threading.Condition()
        self._latest_image: np.ndarray | None = None
        self._latest_depth: np.ndarray | None = None
        self._xrgb: tuple[np.ndarray, float] | None = None
        self._xdepth: tuple[np.ndarray, float] | None = None
        self._latest_odom: tuple[float, float, float] | None = None
        self._xodom_received = 0.0
        self._xframe = ""
        self._latest_costmap: OccupancyGrid | None = None
        self._latest_scan_points: np.ndarray | None = None

        self._state = NavigationState.IDLE
        self._goal_pose: PoseStamped | None = None
        self._goal_reached = False
        self._language_goal = ""
        self._path_guidance: list[tuple[float, float]] | None = None
        self._object_direction: str | None = None
        self._motion_paused = False
        self._exploration_enabled = False
        self._running = False
        self._xepoch = 0
        self._xlast_rgb = -1.0
        self._xplan: MotionPlan | None = None
        self._xplan_source: tuple[str, int] | None = None
        self._pending_plan: tuple[Any, ...] | None = None
        self._plan_generation = 0
        self._latest_traj: np.ndarray | None = None
        self._latest_all_traj: np.ndarray | None = None
        self._latest_all_vals: np.ndarray | None = None
        self._traj_timestamp = 0.0
        self._threads: list[threading.Thread] = []
        self._debug_publisher = _RosDebugPublisher(self.debug_mode)

        self._explore_trail: list[tuple[float, float]] = []
        self._explore_trail_last_pos: tuple[float, float] | None = None
        self._stats_lock = threading.Lock()
        self._stats_started = time.monotonic()
        self._stage_stats: dict[str, list[float]] = {}
        self._outcomes: dict[str, int] = {}
        self._inference_count = 0
        self._last_error_log = 0.0
        super().__init__(**kwargs)

    @rpc
    def start(self) -> None:
        super().start()
        self._xclient.verify_backend()
        if self._intrinsic is None:
            self._intrinsic = np.array(
                [[460.0, 0.0, 320.0], [0.0, 460.0, 240.0], [0.0, 0.0, 1.0]]
            )
        self._xclient.initialize(self._intrinsic)
        self._disposables.add(Disposable(self.color_image.subscribe(self._on_image)))
        self._disposables.add(Disposable(self.navdp_image.subscribe(self._on_navdp_image)))
        self._disposables.add(Disposable(self.depth_image.subscribe(self._on_depth)))
        self._disposables.add(Disposable(self.navdp_depth.subscribe(self._on_navdp_depth)))
        self._disposables.add(Disposable(self.odom.subscribe(self._on_odom)))
        self._disposables.add(Disposable(self.global_costmap.subscribe(self._on_costmap)))
        self._disposables.add(Disposable(self.explore_cmd.subscribe(self._on_explore_cmd)))
        self._disposables.add(
            Disposable(self.stop_explore_cmd.subscribe(self._on_stop_explore_cmd))
        )
        self._disposables.add(
            Disposable(self.realsense_camera_info.subscribe(self._on_camera_info))
        )
        self._running = True
        self._stats_started = time.monotonic()
        self._threads = [
            threading.Thread(target=self._inference_loop, daemon=True, name="xnavdp-inference"),
            threading.Thread(target=self._planning_loop, daemon=True, name="xnavdp-mpc"),
            threading.Thread(target=self._control_loop, daemon=True, name="xnavdp-control"),
        ]
        for thread in self._threads:
            thread.start()
        logger.info(
            "XNavDP started release=%s server=%s debug=%s inference_hz=%.1f control_hz=%.1f",
            self._xclient.health.get("release"),
            self._navdp_url,
            self.debug_mode,
            1.0 / self._xperiod,
            self._tick_rate_hz,
        )

    @rpc
    def stop(self) -> None:
        self._running = False
        with self._plan_cv:
            self._pending_plan = None
            self._plan_cv.notify_all()
        for thread in self._threads:
            thread.join(timeout=3.0)
        self.cmd_vel.publish(Twist())
        self._debug_publisher.close()
        self._xclient.close()
        super().stop()

    def _on_image(self, img: Image) -> None:
        with self._lock:
            self._latest_image = _image_to_bgr(img)

    def _on_navdp_image(self, img: Image) -> None:
        bgr = _image_to_bgr(img)
        with self._lock:
            self._latest_image = bgr
        with self._xlock:
            self._xrgb = (bgr, float(img.ts))

    def _on_depth(self, img: Image) -> None:
        with self._lock:
            self._latest_depth = img.data

    def _on_navdp_depth(self, img: Image) -> None:
        with self._lock:
            self._latest_depth = img.data
        with self._xlock:
            self._xdepth = (img.data, float(img.ts))

    def _on_odom(self, odom: PoseStamped) -> None:
        with self._lock:
            self._latest_odom = _pose_to_xyyaw(odom)
            self._xframe = odom.frame_id
            self._xodom_received = time.monotonic()

    def _on_costmap(self, costmap: OccupancyGrid) -> None:
        with self._lock:
            self._latest_costmap = costmap
        self._collision_guard._latest_costmap = costmap

    def _on_camera_info(self, info: CameraInfo) -> None:
        if self._cam_intrinsic is None:
            self._intrinsic = np.asarray(info.K, dtype=float).reshape(3, 3)

    def _on_explore_cmd(self, _: Bool) -> None:
        self._exploration_enabled = True
        self.set_language_goal("explore the environment")

    def _on_stop_explore_cmd(self, _: Bool) -> None:
        self._exploration_enabled = False
        self.cancel_goal()

    @rpc
    def set_goal(self, goal: PoseStamped) -> bool:
        return self._accept_goal(goal)

    @rpc
    def set_point_goal(self, goal: PoseStamped, valid_for_s: float = 2.0) -> bool:
        del valid_for_s
        return self._accept_goal(goal)

    def _accept_goal(self, goal: PoseStamped) -> bool:
        if not self._xframe or goal.frame_id != self._xframe:
            return False
        if not np.all(np.isfinite([goal.position.x, goal.position.y])):
            return False
        with self._xlock:
            self._goal_pose = goal
            self._goal_reached = False
            self._state = NavigationState.FOLLOWING_PATH
            self._xepoch += 1
            self._xplan = None
        return True

    @rpc
    def set_language_goal(self, goal: str) -> bool:
        if goal.strip().lower() != "explore the environment":
            logger.warning("XNavDP needs coordinate goals; text is handled by VLN skills")
            return False
        with self._xlock:
            self._language_goal = goal
            self._goal_pose = None
            self._goal_reached = False
            self._state = NavigationState.FOLLOWING_PATH
            self._xepoch += 1
            self._xplan = None
        return True

    @rpc
    def set_reference_image(self, image_bgr: np.ndarray) -> bool:
        logger.warning("XNavDP supports point goals only; use set_point_goal")
        return False

    @rpc
    def cancel_goal(self) -> bool:
        with self._xlock:
            self._goal_pose = None
            self._language_goal = ""
            self._path_guidance = None
            self._goal_reached = False
            self._state = NavigationState.IDLE
            self._xepoch += 1
            self._xplan = None
        self.cmd_vel.publish(Twist())
        return True

    @rpc
    def pause_motion(self) -> bool:
        with self._xlock:
            self._motion_paused = True
            self._xepoch += 1
            self._xplan = None
        self.cmd_vel.publish(Twist())
        return True

    @rpc
    def resume_motion(self) -> bool:
        self._motion_paused = False
        return True

    @rpc
    def set_path_guidance(self, waypoints: list[list[float]]) -> bool:
        with self._xlock:
            self._path_guidance = [(float(w[0]), float(w[1])) for w in waypoints]
        return True

    @rpc
    def clear_path_guidance(self) -> bool:
        with self._xlock:
            self._path_guidance = None
        return True

    @rpc
    def set_object_direction(self, direction: str) -> None:
        self._object_direction = direction

    @rpc
    def clear_object_direction(self) -> None:
        self._object_direction = None

    @rpc
    def set_scan_points(self, points: list[list[float]]) -> None:
        with self._lock:
            self._latest_scan_points = np.asarray(points, dtype=np.float32)

    @rpc
    def set_costmap(self, costmap: OccupancyGrid) -> None:
        self._on_costmap(costmap)

    @rpc
    def get_state(self) -> NavigationState:
        return self._state

    @rpc
    def is_goal_reached(self) -> bool:
        return self._goal_reached

    @rpc
    def get_navdp_state(self) -> str:
        return "IDLE" if self._state == NavigationState.IDLE else "FOLLOWING_PATH"

    @rpc
    def get_language_goal(self) -> str:
        return self._language_goal

    @rpc
    def get_escape_attempt_count(self) -> int:
        return 0

    @rpc
    def clear_exploration_trail(self) -> bool:
        self._explore_trail.clear()
        self._explore_trail_last_pos = None
        return True

    @rpc
    def get_explore_trail(self) -> list[tuple[float, float]]:
        return list(self._explore_trail)

    @rpc
    def get_exploration_stats(self) -> dict[str, Any]:
        trail = list(self._explore_trail)
        trail_distance = sum(
            math.dist(trail[i - 1], trail[i]) for i in range(1, len(trail))
        )
        observed = 0.0
        costmap = self._latest_costmap
        if costmap is not None and costmap.grid.size:
            observed = float(np.count_nonzero(costmap.grid != CostValues.UNKNOWN)) / costmap.grid.size
        return {
            "trail_length": len(trail),
            "trail_distance_m": round(trail_distance, 2),
            "coverage_radius_m": 0.0,
            "observed_fraction": round(observed, 4),
            "costmap_available": costmap is not None,
        }

    def _snapshot(self) -> _Snapshot | None:
        with self._xlock:
            rgb, depth, epoch = self._xrgb, self._xdepth, self._xepoch
            final = self._goal_pose
            path = list(self._path_guidance or [])
        if rgb is None or depth is None or rgb[0].shape[:2] != depth[0].shape:
            return None
        if not np.any(np.isfinite(depth[0]) & (depth[0] > 0)):
            return None
        with self._lock:
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
        if target is None:
            return None
        if final is None and math.dist(target, pose[0][:2]) < 0.2:
            return None
        return _Snapshot(rgb, depth, pose, target, epoch)

    def _inference_loop(self) -> None:
        self._xclient.verify_backend()
        while self._running:
            started = time.monotonic()
            snapshot = self._snapshot()
            if self._state == NavigationState.IDLE or self._motion_paused or snapshot is None:
                time.sleep(0.05)
                continue
            rgb, depth, pose, target, epoch = (
                snapshot.rgb, snapshot.depth, snapshot.pose, snapshot.target, snapshot.epoch
            )
            if rgb[1] <= self._xlast_rgb:
                time.sleep(0.01)
                continue
            try:
                point = local_goal(target, *pose)
                q = Quaternion.from_rotation_matrix(pose[1])
                quaternion = np.array([q.x, q.y, q.z, q.w])
                traj, all_traj, values = self._xclient.pointgoal_step(
                    rgb[0], depth[0], point, pose[0], quaternion,
                    stamp=rgb[1], frame_id=self._xframe,
                )
                pose = self._xclient.last_pose
                point = self._xclient.last_local_goal
                q = Quaternion.from_rotation_matrix(pose[1])
                quaternion = np.array([q.x, q.y, q.z, q.w])
                latest = self._snapshot()
                if (
                    latest is None
                    or latest.epoch != epoch
                    or self._state == NavigationState.IDLE
                    or self._motion_paused
                ):
                    self._count("result_stale")
                    continue
                trace_id = self._xclient.last_trace_id
                self._debug_publisher.publish(
                    traj,
                    all_traj,
                    values,
                    point,
                    pose[0],
                    quaternion,
                    self._xclient.last_http_seconds,
                    trace_id,
                )
                if self.debug_mode:
                    logger.info(
                        "[XNavDP-Trace] request_id=%s request_ms=%.3f points=%d candidates=%d",
                        trace_id,
                        self._xclient.last_http_seconds * 1000.0,
                        traj.shape[1],
                        all_traj.shape[1],
                    )
                source = (trace_id, self._xclient.last_result_at_ns)
                self._queue_plan((traj, all_traj, values, pose, epoch, source))
                self._xlast_rgb = rgb[1]
                with self._stats_lock:
                    self._inference_count += 1
            except Exception as exc:
                self._count("inference_error")
                if time.monotonic() - self._last_error_log >= 5.0:
                    self._last_error_log = time.monotonic()
                    logger.warning("XNavDP pointgoal failed: %s", exc)
                time.sleep(0.5)
            time.sleep(max(0.0, self._xperiod - (time.monotonic() - started)))

    def _queue_plan(self, job: tuple[Any, ...]) -> None:
        with self._plan_cv:
            self._plan_generation += 1
            self._pending_plan = (self._plan_generation, job)
            self._plan_cv.notify()

    def _planning_loop(self) -> None:
        while True:
            with self._plan_cv:
                while self._running and self._pending_plan is None:
                    self._plan_cv.wait(timeout=0.2)
                if not self._running:
                    return
                generation, job = self._pending_plan
                self._pending_plan = None
            traj, all_traj, values, captured_pose, epoch, source = job
            try:
                with self._lock:
                    current = _pose_from_xyyaw(self._latest_odom)
                latest = self._snapshot()
                if (
                    current is None
                    or latest is None
                    or latest.epoch != epoch
                    or self._state == NavigationState.IDLE
                    or self._motion_paused
                ):
                    self._count("plan_stale")
                    continue
                local_path = (
                    traj[0] @ captured_pose[1].T + captured_pose[0] - current[0]
                ) @ current[1]
                plan_started = time.monotonic()
                commands = solve_mpc(
                    local_path,
                    self._mpc_horizon,
                    1.0 / self._tick_rate_hz,
                    self._mpc_desired_v,
                    self._mpc_v_max,
                    min(self._mpc_w_max, self._xmax_angular_speed),
                )
                world_path = local_path @ current[1].T + current[0]
                plan = MotionPlan(commands, world_path, plan_started, 1.0 / self._tick_rate_hz)
                with self._plan_cv:
                    if generation != self._plan_generation:
                        self._count("plan_superseded")
                        continue
                    with self._xlock:
                        if (
                            not self._running
                            or self._xepoch != epoch
                            or self._motion_paused
                            or self._state == NavigationState.IDLE
                        ):
                            self._count("plan_stale")
                            continue
                        self._xplan = plan
                        self._xplan_source = source
                        with self._infer_lock:
                            self._latest_traj = local_path
                            self._latest_all_traj = (
                                all_traj[0] @ captured_pose[1].T
                                + captured_pose[0]
                                - current[0]
                            ) @ current[1]
                            self._latest_all_vals = values
                            self._traj_timestamp = time.time()
                if self.debug_mode and source:
                    logger.info(
                        "[XNavDP-Trace] request_id=%s plan_accepted after_result_ms=%.3f",
                        source[0],
                        (time.monotonic_ns() - source[1]) / 1e6,
                    )
            except Exception:
                self._count("planning_error")
                logger.exception("XNavDP MPC planning failed")

    def _control_loop(self) -> None:
        period = 1.0 / self._tick_rate_hz
        while self._running:
            started = time.monotonic()
            try:
                self._tick()
            except Exception:
                logger.exception("XNavDP control tick failed")
                self._stop("tick_error")
            self._record("tick", time.monotonic() - started)
            self._report_stats()
            time.sleep(max(0.0, period - (time.monotonic() - started)))

    def _tick(self) -> None:
        if self._state == NavigationState.IDLE or self._motion_paused:
            self._stop("idle")
            return
        if time.monotonic() - self._xodom_received > 0.5:
            self._stop("odom_stale")
            return
        snapshot = self._snapshot()
        if snapshot is None:
            self._stop("snapshot_missing")
            return
        with self._lock:
            odom = self._latest_odom
        if odom is None:
            self._stop("odom_missing")
            return
        with self._xlock:
            final = self._goal_pose
            plan = self._xplan
        if final is not None and math.dist(
            (final.position.x, final.position.y), (odom[0], odom[1])
        ) < 0.2:
            self._goal_reached = True
            self.cancel_goal()
            return
        self._update_explore_trail(odom[0], odom[1])
        if plan is None:
            self._stop("no_plan")
            return
        now = time.monotonic()
        v, w = plan.command(now, self._xdecel_start_s, self._xstop_s - self._xdecel_start_s)
        if now - plan.started >= (
            min(self._xdecel_start_s, len(plan.commands) * plan.dt)
            + self._xstop_s
            - self._xdecel_start_s
        ):
            self._stop("expired")
            return
        pose = _pose_from_xyyaw(odom)
        assert pose is not None
        traj = (plan.world_path - pose[0]) @ pose[1]
        with self._lock:
            costmap, scan = self._latest_costmap, self._latest_scan_points
        blocked = collision_veto(
            self._collision_guard,
            odom,
            costmap,
            scan,
            snapshot.depth[0],
            v,
            w,
        )
        rotate_only = False
        if blocked is not None and abs(w) > 1e-3:
            rotate_reason = collision_veto(
                self._collision_guard,
                odom,
                costmap,
                scan,
                snapshot.depth[0],
                0.0,
                w,
            )
            rotate_only = rotate_reason is None
        with self._xlock:
            if self._xplan is not plan or self._motion_paused or self._state == NavigationState.IDLE:
                self._count("superseded")
                return
        # Publish exactly the command that was collision-checked.
        if blocked is not None and not rotate_only:
            v, w = 0.0, 0.0
            self._count("collision_" + blocked)
        elif rotate_only:
            v = 0.0
            self._count("rotate_only")
        else:
            self._count("tracking")
        w = float(np.clip(w, -self._xmax_angular_speed, self._xmax_angular_speed))
        ctx = _TickContext(
            image=snapshot.rgb[0],
            odom=odom,
            depth=snapshot.depth[0],
            traj=traj,
            waypoints=traj[:, :2],
            v=v,
            w=w,
        )
        self._tick_publish(ctx)

    def _tick_publish(self, ctx: _TickContext) -> None:
        self.cmd_vel.publish(
            Twist(
                linear=Vector3(x=float(ctx.v), y=0.0, z=0.0),
                angular=Vector3(x=0.0, y=0.0, z=float(ctx.w)),
            )
        )
        if ctx.waypoints is not None and len(ctx.waypoints):
            ox, oy, yaw = ctx.odom
            c, s = math.cos(yaw), math.sin(yaw)
            poses = [
                PoseStamped(position=[ox + c * x - s * y, oy + s * x + c * y, 0.0])
                for x, y in ctx.waypoints
            ]
            self.navdp_path.publish(Path(poses=poses, frame_id="world"))

    def _stop(self, reason: str) -> None:
        self._count(reason)
        self.cmd_vel.publish(Twist())

    def _update_explore_trail(self, x: float, y: float) -> None:
        if self._explore_trail_last_pos is None or math.dist(
            (x, y), self._explore_trail_last_pos
        ) >= 0.5:
            self._explore_trail.append((x, y))
            self._explore_trail_last_pos = (x, y)
            if len(self._explore_trail) > 2000:
                self._explore_trail = self._explore_trail[-2000:]

    def _count(self, reason: str) -> None:
        with self._stats_lock:
            self._outcomes[reason] = self._outcomes.get(reason, 0) + 1

    def _record(self, stage: str, duration: float) -> None:
        with self._stats_lock:
            stat = self._stage_stats.setdefault(stage, [0.0, 0.0, 0.0])
            stat[0] += 1
            stat[1] += duration
            stat[2] = max(stat[2], duration)

    def _report_stats(self) -> None:
        now = time.monotonic()
        with self._stats_lock:
            if now - self._stats_started < 5.0:
                return
            elapsed = now - self._stats_started
            stages = self._stage_stats
            outcomes = self._outcomes
            inference_count = self._inference_count
            self._stage_stats = {}
            self._outcomes = {}
            self._inference_count = 0
            self._stats_started = now
        timings = {
            name: {
                "n": int(v[0]),
                "mean_ms": round(1000 * v[1] / v[0], 2),
                "max_ms": round(1000 * v[2], 2),
            }
            for name, v in stages.items()
        }
        logger.info(
            "[XNavDP-Perf] window_s=%.2f tick_hz=%.2f inference_hz=%.2f stages=%s outcomes=%s",
            elapsed,
            stages.get("tick", [0])[0] / elapsed,
            inference_count / elapsed,
            timings,
            outcomes,
        )


xnavdp_navigator = XNavDPNavigator.blueprint
