"""ROS/JSON adapters; algorithms consume plain arrays, as in simple."""

import json
import math
from dataclasses import dataclass
from enum import IntEnum
from types import SimpleNamespace

import numpy as np


class CostValues(IntEnum):
    UNKNOWN = -1


class OccupancyGrid:
    """Only the original collision guard's grid interface, without DimOS messages."""

    def __init__(self, msg):
        self.width, self.height = msg.info.width, msg.info.height
        self.resolution = msg.info.resolution
        if self.resolution <= 0:
            raise ValueError("invalid costmap resolution")
        self.origin = msg.info.origin
        self.grid = np.asarray(msg.data, dtype=np.int8).reshape(self.height, self.width)

    def world_to_grid(self, point):
        return SimpleNamespace(
            x=(point[0] - self.origin.position.x) / self.resolution,
            y=(point[1] - self.origin.position.y) / self.resolution,
        )


def yaw_pose(position, quaternion):
    p, q = np.asarray(position, dtype=float), np.asarray(quaternion, dtype=float)
    if (
        p.shape != (3,)
        or q.shape != (4,)
        or not np.isfinite(p).all()
        or not np.isfinite(q).all()
    ):
        raise ValueError("invalid pose")
    if np.dot(q, q) < 1e-12:
        raise ValueError("zero pose quaternion")
    x, y, z, w = q
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([p[0], p[1], 0.0]), np.array(
        [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
    )


def odom_pose(msg):
    p, q = msg.pose.position, msg.pose.orientation
    pose = yaw_pose([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
    yaw = math.atan2(pose[1][1, 0], pose[1][0, 0])
    if not msg.header.frame_id:
        raise ValueError("missing odom frame")
    return (p.x, p.y, yaw), msg.header.frame_id


@dataclass(frozen=True)
class Result:
    stamp_ns: int
    trajectory: np.ndarray
    candidates: np.ndarray
    values: np.ndarray
    pose: tuple
    goal_world: np.ndarray
    world_frame: str


def decode_result(data):
    item = json.loads(data)
    if not isinstance(item, dict) or item.get("status") != "success":
        raise ValueError(
            item.get("error", "inference failed")
            if isinstance(item, dict)
            else "invalid result"
        )
    stamp = item["request_stamp_ns"]
    if type(stamp) is not int or stamp <= 0:
        raise ValueError("invalid request_stamp_ns")
    if item["trajectory_frame"] != "base_link" or item["goal_frame"] != "base_link":
        raise ValueError("expected base_link trajectory and goal")
    if not isinstance(item["world_frame"], str) or not item["world_frame"]:
        raise ValueError("missing world frame")
    trajectory = np.asarray(item["trajectory"], dtype=float)
    candidates = np.asarray(item["all_trajectory"], dtype=float)
    values = np.asarray(item["all_values"], dtype=float)
    # The same result shape/finite checks as simple's pointgoal_step.
    if (
        trajectory.ndim != 3
        or trajectory.shape[0] != 1
        or trajectory.shape[2] != 3
        or not trajectory.shape[1]
        or candidates.ndim != 4
        or candidates.shape[0] != 1
        or candidates.shape[2:] != trajectory.shape[1:]
        or values.size != candidates.shape[1]
        or not all(np.isfinite(a).all() for a in (trajectory, candidates, values))
    ):
        raise ValueError("invalid model paths/candidates/scores")
    pose = yaw_pose(item["robot_pos"], item["robot_quat"])
    goal = np.asarray(item["goal"], dtype=float)
    if goal.shape != (2,) or not np.isfinite(goal).all():
        raise ValueError("invalid goal")
    target = np.r_[goal, 0.0] @ pose[1].T + pose[0]
    return Result(
        stamp, trajectory, candidates, values, pose, target[:2], item["world_frame"]
    )


def decode_depth(msg, scale):
    types = {"16UC1": ("u2", scale), "mono16": ("u2", scale), "32FC1": ("f4", 1.0)}
    if msg.encoding not in types:
        raise ValueError(f"unsupported depth encoding: {msg.encoding}")
    kind, divisor = types[msg.encoding]
    dtype = np.dtype((">" if msg.is_bigendian else "<") + kind)
    if msg.height <= 0 or msg.width <= 0 or msg.step < msg.width * dtype.itemsize:
        raise ValueError("invalid depth dimensions/step")
    if len(msg.data) < msg.height * msg.step:
        raise ValueError("truncated depth data")
    raw = np.ndarray(
        (msg.height, msg.width),
        dtype=dtype,
        buffer=memoryview(msg.data),
        strides=(msg.step, dtype.itemsize),
    )
    return raw.astype(np.float32) / divisor


def decode_lidar(msg, odom, world_frame, *, floor_z=None):
    # minimum publishes x/y/z FLOAT32 in world; honor offsets and row padding.
    fields = {f.name: f for f in msg.fields}
    if any(
        k not in fields or fields[k].datatype != 7 or fields[k].count != 1
        for k in ("x", "y", "z")
    ):
        raise ValueError("expected FLOAT32 XYZ lidar")
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": [(">" if msg.is_bigendian else "<") + "f4"] * 3,
            "offsets": [fields[k].offset for k in ("x", "y", "z")],
            "itemsize": msg.point_step,
        }
    )
    points = np.ndarray(
        (msg.height, msg.width),
        dtype=dtype,
        buffer=memoryview(msg.data),
        strides=(msg.row_step, msg.point_step),
    )
    xy = np.column_stack([points[k].ravel() for k in ("x", "y")])
    valid = np.isfinite(xy).all(axis=1)
    if floor_z is not None and msg.header.frame_id == world_frame:
        height = points["z"].ravel() - floor_z
        valid &= (height > 0.08) & (height <= 1.0)
    xy = xy[valid]
    if msg.header.frame_id == "base_link":
        return xy
    if msg.header.frame_id != world_frame or odom is None:
        raise ValueError("lidar frame differs from odom")
    x, y, yaw = odom
    c, s = math.cos(yaw), math.sin(yaw)
    return (xy - [x, y]) @ np.array([[c, -s], [s, c]])
