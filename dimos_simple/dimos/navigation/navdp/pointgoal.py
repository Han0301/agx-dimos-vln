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

"""XNavDP HTTP contract and robot-local point goals. No model dependencies."""

from __future__ import annotations

import json
import math
from typing import Protocol

import cv2
import numpy as np
import requests

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.spec.utils import Spec


class PointGoalNavigatorSpec(Spec, Protocol):
    def set_point_goal(self, goal: PoseStamped, valid_for_s: float = 2.0) -> bool: ...
    def cancel_goal(self) -> bool: ...
    def resume_motion(self) -> bool: ...
    def set_path_guidance(self, waypoints: list[list[float]]) -> bool: ...


def path_lookahead(
    path: list[tuple[float, float]], xy: tuple[float, float], distance: float
) -> tuple[float, float] | None:
    """Select an actual waypoint along a world-frame path, never a unit vector."""
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
    """Official XNavDP uses base-local +X forward, +Y left, metres."""
    world = np.array([world_xy[0], world_xy[1], position[2]], dtype=float)
    result = rotation.T @ (world - position)
    if not np.all(np.isfinite(result)):
        raise ValueError("Nonfinite point goal")
    return result[:2]


class XNavDPClient:
    def __init__(self, url: str, timeout_s: float = 5.0):
        self.url = url.rstrip("/")
        self.timeout_s = timeout_s
        self.session = requests.Session()
        self.session.trust_env = False
        self.initialized = False

    def verify_backend(self) -> None:
        response = self.session.get(self.url + "/health", timeout=(2, 3))
        response.raise_for_status()
        data = response.json()
        if data.get("backend") != "xnavdp" or data.get("goal_frame") != "base_link":
            raise ValueError(
                "Endpoint is not the XNavDP base-frame adapter; original NavDP must stay on its own URL"
            )

    def pointgoal_step(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
        goal: np.ndarray,
        position: np.ndarray,
        quaternion: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        if rgb.shape[:2] != depth.shape or not np.any(np.isfinite(depth) & (depth > 0)):
            raise ValueError("Aligned, nonzero depth is required")
        ok_rgb, jpg = cv2.imencode(".jpg", rgb, [cv2.IMWRITE_JPEG_QUALITY, 80])
        scaled = np.clip(
            np.nan_to_num(depth, nan=0, posinf=0, neginf=0) * 10000, 0, 65535
        ).astype(np.uint16)
        ok_depth, png = cv2.imencode(".png", scaled)
        if not ok_rgb or not ok_depth:
            raise ValueError("Image encoding failed")
        response = self.session.post(
            self.url + "/pointgoal_step",
            files={
                "image": ("image.jpg", jpg.tobytes(), "image/jpeg"),
                "depth": ("depth.png", png.tobytes(), "image/png"),
            },
            data={
                "goal_data": json.dumps(
                    {"goal_x": [float(goal[0])], "goal_y": [float(goal[1])]}
                ),
                "state_data": json.dumps(
                    {
                        "robot_pos": [position.tolist()],
                        "robot_quat": [quaternion.tolist()],
                    }
                ),
            },
            timeout=(2, self.timeout_s),
        )
        if not response.ok:
            raise RuntimeError(
                f"XNavDP HTTP {response.status_code}: {response.text[:500]}"
            )
        data = response.json()
        arrays = tuple(
            np.asarray(data[key], dtype=float)
            for key in ("trajectory", "all_trajectory", "all_values")
        )
        traj, candidates, values = arrays
        if (
            traj.ndim != 3
            or traj.shape[0] != 1
            or traj.shape[2] != 3
            or not traj.shape[1]
        ):
            raise ValueError("Invalid trajectory shape")
        if (
            candidates.ndim != 4
            or candidates.shape[0] != 1
            or candidates.shape[2:] != traj.shape[1:]
        ):
            raise ValueError("Invalid candidate trajectories")
        if values.size != candidates.shape[1] or not all(
            np.all(np.isfinite(a)) for a in arrays
        ):
            raise ValueError("Invalid or nonfinite trajectory values")
        return arrays

    def close(self) -> None:
        self.session.close()
