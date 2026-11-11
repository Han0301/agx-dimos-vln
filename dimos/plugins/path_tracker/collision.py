# Extracted from dimos_simple/dimos/navigation/xnavdp/tracking.py (Apache-2.0).
from __future__ import annotations

import math
from typing import Any

import numpy as np

from .messages import CostValues, OccupancyGrid


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
        depth_hard_stop: bool = False,
        **_: Any,
    ) -> None:
        self.cost_threshold = int(cost_threshold)
        self.robot_length = float(robot_length)
        self.robot_half_width = float(robot_half_width)
        self.robot_radius = float(robot_radius)
        self.depth_obstacle_m = float(depth_obstacle_m)
        self.horizon_m = float(horizon_m)
        self.collision_penalty = float(collision_penalty)
        self.depth_hard_stop = depth_hard_stop
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
                severity > 0.5 and (self.depth_hard_stop or forward_fraction > 0.2),
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
                        max_cost = (
                            cost
                            if max_cost == CostValues.UNKNOWN
                            else max(max_cost, cost)
                        )
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
                guard._max_cost_in_obb(
                    costmap, px, py, heading, half_length, half_width
                )
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
