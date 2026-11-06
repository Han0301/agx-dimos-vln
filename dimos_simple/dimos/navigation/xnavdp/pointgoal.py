"""XNavDP point-goal helpers and protocol."""

from __future__ import annotations

import math
from typing import Protocol

import numpy as np

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.spec.utils import Spec


class PointGoalNavigatorSpec(Spec, Protocol):
    def get_navdp_state(self) -> str: ...
    def set_point_goal(self, goal: PoseStamped, valid_for_s: float = 2.0) -> bool: ...
    def cancel_goal(self) -> bool: ...
    def resume_motion(self) -> bool: ...
    def set_path_guidance(self, waypoints: list[list[float]]) -> bool: ...


def path_lookahead(
    path: list[tuple[float, float]], xy: tuple[float, float], distance: float
) -> tuple[float, float] | None:
    if not path:
        return None
    index = min(
        range(len(path)),
        key=lambda i: math.hypot(path[i][0] - xy[0], path[i][1] - xy[1]),
    )
    accumulated = 0.0
    for i in range(index, len(path) - 1):
        length = math.dist(path[i], path[i + 1])
        if length and accumulated + length >= distance:
            fraction = (distance - accumulated) / length
            return tuple(
                path[i][j] + fraction * (path[i + 1][j] - path[i][j]) for j in range(2)
            )
        accumulated += length
    return tuple(path[-1])


def local_goal(
    world_xy: tuple[float, float], position: np.ndarray, rotation: np.ndarray
) -> np.ndarray:
    world = np.array([world_xy[0], world_xy[1], position[2]], dtype=float)
    result = rotation.T @ (world - position)
    if not np.all(np.isfinite(result)):
        raise ValueError("Nonfinite point goal")
    return result[:2]
