"""XNavDP trajectory execution, MPC, and collision checks."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import numpy as np
from scipy.optimize import minimize

from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid


@dataclass(frozen=True)
class MotionPlan:
    commands: np.ndarray
    world_path: np.ndarray
    started: float
    dt: float

    def command(self, now: float, valid_s: float, decel_s: float) -> tuple[float, float]:
        age = max(0.0, now - self.started)
        valid_s = min(valid_s, len(self.commands) * self.dt)
        if age >= valid_s + decel_s:
            return 0.0, 0.0
        index = min(int(min(age, max(0.0, valid_s - 1e-9)) / self.dt), len(self.commands) - 1)
        scale = 1.0 if age < valid_s else max(0.0, 1.0 - (age - valid_s) / max(decel_s, 1e-9))
        v, w = self.commands[index] * scale
        return float(v), float(w)


def solve_mpc(
    path: np.ndarray,
    horizon: int,
    dt: float,
    desired_v: float,
    v_max: float,
    w_max: float,
) -> np.ndarray:
    points = np.asarray(path, dtype=float)[:, :2]
    if len(points) < 2 or not np.isfinite(points).all():
        raise ValueError("Invalid XNavDP selected path")
    points = np.vstack((np.zeros((1, 2)), points))
    distance = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    keep = np.r_[True, np.diff(distance) > 1e-8]
    points, distance = points[keep], distance[keep]
    if len(points) < 2:
        return np.zeros((horizon, 2))
    speed = min(v_max, desired_v)
    arc = np.minimum(np.arange(horizon + 1) * dt * speed, distance[-1])
    reference = np.column_stack([np.interp(arc, distance, points[:, j]) for j in range(2)])

    def objective(flat: np.ndarray) -> tuple[float, np.ndarray]:
        controls = flat.reshape(horizon, 2)
        states = np.zeros((horizon + 1, 3))
        for i, (v, w) in enumerate(controls):
            yaw = states[i, 2]
            states[i + 1] = states[i] + dt * np.array([v * math.cos(yaw), v * math.sin(yaw), w])
        error = states[:, :2] - reference
        cost = 10.0 * np.sum(error**2) + 0.05 * np.sum(controls**2)
        adjoint = np.r_[20.0 * error[-1], 0.0]
        gradient = np.empty_like(controls)
        for i in range(horizon - 1, -1, -1):
            v, _ = controls[i]
            c, s = math.cos(states[i, 2]), math.sin(states[i, 2])
            gradient[i] = 0.1 * controls[i] + dt * np.array(
                [c * adjoint[0] + s * adjoint[1], adjoint[2]]
            )
            adjoint = np.array(
                [
                    adjoint[0],
                    adjoint[1],
                    adjoint[2] + dt * v * (-s * adjoint[0] + c * adjoint[1]),
                ]
            )
            adjoint[:2] += 20.0 * error[i]
        return float(cost), gradient.ravel()

    initial = np.zeros((horizon, 2))
    initial[:, 0] = math.copysign(min(speed, 0.05), points[1, 0])
    initial[:, 1] = np.clip(
        math.atan2(points[1, 1], abs(points[1, 0]) + 1e-8), -w_max, w_max
    )
    result = minimize(
        objective,
        initial.ravel(),
        jac=True,
        method="L-BFGS-B",
        bounds=[(-v_max, v_max), (-w_max, w_max)] * horizon,
        options={"maxiter": 100, "ftol": 1e-8, "gtol": 1e-5},
    )
    if not result.success or not np.isfinite(result.x).all():
        raise RuntimeError(f"XNavDP MPC failed: {result.message}")
    return result.x.reshape(horizon, 2)


class XNavDPCollisionGuard:
    """Costmap/depth guard used by XNavDP without navdp_bridge."""

    def __init__(
        self,
        *,
        cost_threshold: int = 50,
        robot_length: float = 0.6,
        robot_half_width: float = 0.15,
        robot_radius: float = 0.30,
        depth_obstacle_m: float = 0.5,
        horizon_m: float = 1.5,
        collision_penalty: float = 1000.0,
        **_: Any,
    ) -> None:
        self.cost_threshold = int(cost_threshold)
        self.robot_length = float(robot_length)
        self.robot_half_width = float(robot_half_width)
        self.robot_radius = float(robot_radius)
        self.depth_obstacle_m = float(depth_obstacle_m)
        self.horizon_m = float(horizon_m)
        self.collision_penalty = float(collision_penalty)
        self.enabled = True
        self._latest_costmap: OccupancyGrid | None = None

    def _depth_forward_cost(
        self, depth_image: np.ndarray, waypoints_base: np.ndarray
    ) -> tuple[float, bool]:
        if waypoints_base.size == 0:
            return 0.0, False
        h, w = depth_image.shape[:2]
        centre = depth_image[int(h * 0.2) : int(h * 0.8), int(w * 0.3) : int(w * 0.7)]
        valid = centre[(centre > 0.05) & np.isfinite(centre)]
        if len(valid) == 0:
            return 0.0, False
        min_depth = float(valid.min())
        endpoint = waypoints_base[-1]
        if endpoint[0] <= 0 or min_depth >= self.depth_obstacle_m * 2.0:
            return 0.0, False
        forward_fraction = min(endpoint[0] / self.horizon_m, 1.0)
        if min_depth < self.depth_obstacle_m:
            severity = 1.0 - min_depth / self.depth_obstacle_m
            return (
                severity * forward_fraction * self.collision_penalty,
                severity > 0.5 and forward_fraction > 0.2,
            )
        severity = 1.0 - (min_depth - self.depth_obstacle_m) / self.depth_obstacle_m
        return severity * forward_fraction * self.collision_penalty * 0.4, False

    def _max_cost_in_obb(
        self,
        costmap: OccupancyGrid,
        wx: float,
        wy: float,
        heading: float,
        half_length: float,
        half_width: float,
    ) -> int:
        cos_h, sin_h = math.cos(heading), math.sin(heading)
        corners = [
            (wx + cos_h * lx - sin_h * ly, wy + sin_h * lx + cos_h * ly)
            for lx, ly in (
                (half_length, half_width),
                (half_length, -half_width),
                (-half_length, half_width),
                (-half_length, -half_width),
            )
        ]
        grid_points = [costmap.world_to_grid((cx, cy, 0.0)) for cx, cy in corners]
        gx_min = max(0, int(math.floor(min(p.x for p in grid_points))))
        gx_max = min(costmap.width - 1, int(math.ceil(max(p.x for p in grid_points))))
        gy_min = max(0, int(math.floor(min(p.y for p in grid_points))))
        gy_max = min(costmap.height - 1, int(math.ceil(max(p.y for p in grid_points))))
        if gx_min > gx_max or gy_min > gy_max:
            return int(CostValues.UNKNOWN)
        max_cost = int(CostValues.UNKNOWN)
        res = costmap.resolution
        ox = costmap.origin.position.x
        oy = costmap.origin.position.y
        for gy in range(gy_min, gy_max + 1):
            for gx in range(gx_min, gx_max + 1):
                dx = ox + (gx + 0.5) * res - wx
                dy = oy + (gy + 0.5) * res - wy
                lx = cos_h * dx + sin_h * dy
                ly = -sin_h * dx + cos_h * dy
                if abs(lx) <= half_length and abs(ly) <= half_width:
                    cost = int(costmap.grid[gy, gx])
                    if cost != CostValues.UNKNOWN:
                        max_cost = cost if max_cost == CostValues.UNKNOWN else max(max_cost, cost)
        return max_cost


def collision_veto(
    guard: XNavDPCollisionGuard,
    odom: tuple[float, float, float],
    costmap: OccupancyGrid | None,
    scan: np.ndarray | None,
    depth: np.ndarray,
    v: float,
    w: float,
    horizon_s: float = 0.8,
) -> str | None:
    if not math.isfinite(v) or not math.isfinite(w):
        return "invalid_command"
    if costmap is None:
        costmap = guard._latest_costmap
    else:
        guard._latest_costmap = costmap
    x, y, yaw = odom
    samples = max(1, math.ceil(horizon_s / 0.1))
    dt = horizon_s / samples
    poses = [(x, y, yaw)]
    for _ in range(samples):
        mid_yaw = yaw + 0.5 * w * dt
        x += v * math.cos(mid_yaw) * dt
        y += v * math.sin(mid_yaw) * dt
        yaw += w * dt
        poses.append((x, y, yaw))

    if costmap is not None:
        low_x = costmap.origin.position.x
        low_y = costmap.origin.position.y
        high_x = low_x + costmap.width * costmap.resolution
        high_y = low_y + costmap.height * costmap.resolution
        half_length = guard.robot_length / 2.0
        half_width = guard.robot_half_width
        for px, py, heading in poses:
            c, s = math.cos(heading), math.sin(heading)
            for lx in (-half_length, half_length):
                for ly in (-half_width, half_width):
                    wx, wy = px + c * lx - s * ly, py + s * lx + c * ly
                    if not (low_x <= wx < high_x and low_y <= wy < high_y):
                        return "map_bounds"
            if (
                guard._max_cost_in_obb(costmap, px, py, heading, half_length, half_width)
                >= guard.cost_threshold
            ):
                return "map"
    elif scan is not None and len(scan):
        ox, oy, oyaw = odom
        c, s = math.cos(oyaw), math.sin(oyaw)
        local = np.column_stack(
            (
                c * (np.array([p[0] for p in poses]) - ox)
                + s * (np.array([p[1] for p in poses]) - oy),
                -s * (np.array([p[0] for p in poses]) - ox)
                + c * (np.array([p[1] for p in poses]) - oy),
            )
        )
        for px, py in local:
            if np.any(
                np.sum((scan[:, :2] - np.array([px, py])) ** 2, axis=1)
                < guard.robot_radius**2
            ):
                return "lidar"
    if v > 0 and guard.depth_obstacle_m > 0:
        short_path = np.array([[0.0, 0.0], [v * horizon_s, 0.0]])
        if guard._depth_forward_cost(depth, short_path)[1]:
            return "depth"
    return None
