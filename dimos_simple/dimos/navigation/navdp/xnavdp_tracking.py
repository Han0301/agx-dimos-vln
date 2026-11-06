"""XNavDP signed MPC execution, independent of legacy NavDP recovery.

Uses the upstream unicycle model and position/control cost (Qxy=10, R=.05).
SciPy bounded shooting replaces acados; this is not the upstream solver.
Planning runs outside the control loop. No proportional fallback on failure.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import numpy as np
from scipy.optimize import minimize


@dataclass(frozen=True)
class MotionPlan:
    commands: np.ndarray
    world_path: np.ndarray
    started: float
    dt: float

    def command(self, now: float, valid_s: float, decel_s: float) -> tuple[float, float]:
        age = max(0.0, now - self.started)
        # Consume the sequence until valid_s, then decay that boundary command.
        # A short sequence starts deceleration earlier; never loop it.
        valid_s = min(valid_s, len(self.commands) * self.dt)
        if age >= valid_s + decel_s:
            return 0.0, 0.0
        index = min(int(min(age, max(0.0, valid_s - 1e-9)) / self.dt), len(self.commands) - 1)
        scale = 1.0 if age < valid_s else max(0.0, 1.0 - (age - valid_s) / max(decel_s, 1e-9))
        v, w = self.commands[index] * scale
        return float(v), float(w)


def solve_mpc(path: np.ndarray, horizon: int, dt: float, desired_v: float,
              v_max: float, w_max: float) -> np.ndarray:
    """Finite-horizon, signed unicycle tracking with analytic cost gradient."""
    points = np.asarray(path, dtype=float)[:, :2]
    if len(points) < 2 or not np.isfinite(points).all():
        raise ValueError("Invalid XNavDP selected path")
    points = np.vstack((np.zeros((1, 2)), points))
    distance = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    keep = np.r_[True, np.diff(distance) > 1e-8]
    points, distance = points[keep], distance[keep]
    if len(points) < 2:
        return np.zeros((horizon, 2))
    # Keep nominal speed; the reference still stops at the actual path endpoint.
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
        cost = 10.0 * np.sum(error ** 2) + 0.05 * np.sum(controls ** 2)
        adjoint = np.r_[20.0 * error[-1], 0.0]
        gradient = np.empty_like(controls)
        for i in range(horizon - 1, -1, -1):
            v, _ = controls[i]
            c, s = math.cos(states[i, 2]), math.sin(states[i, 2])
            gradient[i] = 0.1 * controls[i] + dt * np.array([c * adjoint[0] + s * adjoint[1], adjoint[2]])
            adjoint = np.array([adjoint[0], adjoint[1], adjoint[2] + dt * v * (-s * adjoint[0] + c * adjoint[1])])
            adjoint[:2] += 20.0 * error[i]
        return float(cost), gradient.ravel()

    initial = np.zeros((horizon, 2))
    # A small signed seed breaks the zero-control stationary solution for turns.
    initial[:, 0] = math.copysign(min(speed, 0.05), points[1, 0])
    initial[:, 1] = np.clip(math.atan2(points[1, 1], abs(points[1, 0]) + 1e-8), -w_max, w_max)
    result = minimize(objective, initial.ravel(), jac=True, method="L-BFGS-B",
                      bounds=[(-v_max, v_max), (-w_max, w_max)] * horizon,
                      options={"maxiter": 100, "ftol": 1e-8, "gtol": 1e-5})
    if not result.success or not np.isfinite(result.x).all():
        raise RuntimeError(f"XNavDP MPC failed: {result.message}")
    return result.x.reshape(horizon, 2)


def collision_veto(
    selector: Any,
    odom: tuple[float, float, float],
    costmap: Any,
    scan: np.ndarray | None,
    depth: np.ndarray,
    v: float,
    w: float,
    horizon_s: float = 0.8,
) -> str | None:
    """Check the swept footprint of the commanded motion; return blocking source.

    The prediction is a safety check, not a second trajectory selector.  Sample
    pure rotations as well as translations so an unsafe turn cannot slip through.
    """
    if not math.isfinite(v) or not math.isfinite(w):
        return "invalid_command"
    if costmap is None:
        costmap = selector._latest_costmap
    else:
        selector._latest_costmap = costmap
    x, y, yaw = odom
    samples = max(1, math.ceil(horizon_s / 0.1))
    dt = horizon_s / samples
    poses = [(x, y, yaw)]
    for _ in range(samples):
        # Midpoint integration also handles a curved reverse command.
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
        for px, py, heading in poses:
            # Keep the old out-of-map veto. A centre inside the grid may have
            # body corners outside; reject that case too.
            half_length = selector.robot_length / 2.0
            half_width = selector.robot_half_width
            c, s = math.cos(heading), math.sin(heading)
            for lx in (-half_length, half_length):
                for ly in (-half_width, half_width):
                    wx, wy = px + c * lx - s * ly, py + s * lx + c * ly
                    if not (low_x <= wx < high_x and low_y <= wy < high_y):
                        return "map_bounds"
            if selector._max_cost_in_obb(
                costmap, px, py, heading, half_length, half_width
            ) >= selector.cost_threshold:
                return "map"
    elif scan is not None and len(scan):
        # Radar point coordinates are robot-local; include rear points for
        # reverse commands, unlike the old forward-only fallback.
        ox, oy, oyaw = odom
        c, s = math.cos(oyaw), math.sin(oyaw)
        local = np.column_stack((
            c * (np.array([p[0] for p in poses]) - ox)
            + s * (np.array([p[1] for p in poses]) - oy),
            -s * (np.array([p[0] for p in poses]) - ox)
            + c * (np.array([p[1] for p in poses]) - oy),
        ))
        for px, py in local:
            if np.any(np.sum((scan[:, :2] - np.array([px, py])) ** 2, axis=1)
                      < selector.robot_radius ** 2):
                return "lidar"
    if v > 0 and selector.depth_obstacle_m > 0:
        # Preserve the existing forward-depth veto with the short action path.
        short_path = np.array([[0.0, 0.0], [v * horizon_s, 0.0]])
        if selector._depth_forward_cost(depth, short_path)[1]:
            return "depth"
    return None
