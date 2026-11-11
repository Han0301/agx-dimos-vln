"""Turn in place to scan nearby headings and approach visible targets with bad depth."""

import logging
import math
import threading
import time
from functools import partial
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.plugins.path_tracker.messages import odom_pose
from dimos.plugins.runtime_log import event
from dimos.plugins.vln_task.observation import TRACKING_MAX_AGE_S
from dimos.plugins.vln_task.preview import optical_ray
from dimos.plugins.xnavdp.client import stamp_ns

from .search import ExplorationUnavailable

logger = logging.getLogger(__name__)


class ExplorationCloud:
    """Latest world-frame Go2 voxel endpoints, used as cell evidence."""

    def __init__(self, node, topic):
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import PointCloud2

        self.lock = threading.Lock()
        self.latest = None
        self.subscription = None if topic is None else node.create_subscription(
            PointCloud2, topic, self._receive, qos_profile_sensor_data
        )

    def _receive(self, message):
        stamp = stamp_ns(message.header)
        if stamp <= 0:
            return
        with self.lock:
            if self.latest is None or stamp > self.latest[1]:
                self.latest = message, stamp, time.monotonic()

    def ready(self):
        with self.lock:
            return self.latest is not None and time.monotonic() - self.latest[2] <= 0.75

    def read(self, odom):
        from .occupancy import decode_world_lidar

        with self.lock:
            sample = self.latest
        if sample is None:
            event(logger, "exploration_lidar_rejected", reason="missing")
            return None
        message, stamp, received = sample
        age = time.monotonic() - received
        odom_stamp = stamp_ns(odom.header)
        gap = abs(stamp - odom_stamp)
        if age > 0.75 or gap > 750_000_000:
            event(
                logger, "exploration_lidar_rejected",
                reason="stale" if age > 0.75 else "odom_stamp_gap",
                lidar_stamp_ns=stamp, odom_stamp_ns=odom_stamp,
                age_ms=age * 1000, gap_ms=gap / 1e6,
            )
            return None
        try:
            points = decode_world_lidar(message, odom.header.frame_id)
        except (ValueError, TypeError) as exc:
            logger.warning("Invalid exploration LiDAR sample: %s", type(exc).__name__)
            event(logger, "exploration_lidar_rejected", reason="decode_error",
                  error_type=type(exc).__name__, lidar_stamp_ns=stamp)
            return None
        if not len(points):
            event(logger, "exploration_lidar_rejected", reason="empty", lidar_stamp_ns=stamp)
            return None
        return SimpleNamespace(
            points_xyz=points, pose_z=odom.pose.position.z,
            frame=odom.header.frame_id, stamp_ns=stamp,
        )


class RotationSweep:
    """Sample five headings spaced 60 degrees from the first observation."""

    def __init__(self, observe, newest_stamp, turn, first, cancel, *, settle_s=0.15):
        self.observe = observe
        self.newest_stamp = newest_stamp
        self.turn = turn
        pose, self.world_frame = odom_pose(first.odom)
        self.initial_yaw = pose[2]
        self.headings = iter(
            math.remainder(self.initial_yaw + math.radians(angle), math.tau)
            for angle in range(60, 360, 60)
        )
        self.cancel = cancel
        self.settle_s = settle_s

    def __call__(self, previous, remaining, on_progress):
        yaw = next(self.headings, None)
        if yaw is None:
            return None
        leg = {
            "leg_kind": "rotation", "source_stamp_ns": previous.stamp_ns,
            "goal_yaw": yaw, "world_frame": self.world_frame,
        }
        event(logger, "exploration_leg_started", **leg)
        reason = self.turn(yaw, self.world_frame, remaining)
        remaining()
        if reason is not None:
            event(logger, "exploration_leg_finished", **leg, reason=reason)
            raise ExplorationUnavailable(reason)
        self.cancel.wait(min(self.settle_s, remaining()))

        def accept_odom(odom):
            pose, frame = odom_pose(odom)
            return frame == self.world_frame and abs(
                math.remainder(yaw - pose[2], math.tau)
            ) <= math.radians(5)

        stopped = self.observe(
            remaining, after_stamp_ns=max(previous.stamp_ns, self.newest_stamp()),
            accept_odom=accept_odom, max_age_s=TRACKING_MAX_AGE_S,
        )
        event(logger, "exploration_leg_finished", **leg, reason="new_view",
              stopped_stamp_ns=stopped.stamp_ns)
        return stopped


class DirectedExploration:
    def __init__(
        self, observe, newest_stamp, approach, mapper, planner, cancel,
        *, cloud=None, leg_timeout_s=7.0, settle_s=0.15,
    ):
        self.observe = observe
        self.newest_stamp = newest_stamp
        self.approach = approach
        self.mapper = mapper
        self.planner = planner
        self.cancel = cancel
        self.cloud = cloud
        self.leg_timeout_s = leg_timeout_s
        self.settle_s = settle_s
        self.leg_id = 0
        self.lidar_stamp_ns = None

    def _begin_leg(self, kind, frame):
        self.leg_id += 1
        leg = {"leg_id": self.leg_id, "leg_kind": kind}
        event(
            logger, "exploration_leg_started", **leg, source_stamp_ns=frame.stamp_ns,
            observation_id=getattr(frame, "observation_id", None),
        )
        return leg

    def _map(self, observation):
        pose, frame = odom_pose(observation.odom)
        self.mapper.update(observation)
        if self.cloud is not None:
            sample = self.cloud.read(observation.odom)
            if sample is None:
                raise ExplorationUnavailable("no_fresh_lidar")
            self.lidar_stamp_ns = sample.stamp_ns
            self.mapper.update_cloud(
                sample.points_xyz, pose_z=sample.pose_z,
                frame=sample.frame, stamp_ns=sample.stamp_ns,
            )
        grid = self.mapper.snapshot(frame=frame, require_cloud=self.cloud is not None)
        if grid is None or grid.stamp_ns != observation.stamp_ns:
            raise ExplorationUnavailable("no_valid_exploration_map")
        return grid, pose

    def _accept_path(self, _, result, check, *, leg):
        latest = self.observe(check, max_age_s=TRACKING_MAX_AGE_S)
        grid, pose = self._map(latest)
        if grid.frame != result.world_frame:
            reason = "world_frame_mismatch"
        else:
            world_path = result.trajectory[0] @ result.pose[1].T + result.pose[0]
            if self.planner.path_is_clear(
                grid.grid, origin_xy=grid.origin_xy, resolution=grid.resolution,
                robot_xy=pose[:2], path_xy=world_path[:, :2],
            ):
                return True
            reason = "insufficient_clearance"
        event(
            logger, "exploration_path_rejected", **leg, reason=reason,
            request_stamp_ns=getattr(result, "stamp_ns", None), grid_stamp_ns=grid.stamp_ns,
            lidar_stamp_ns=self.lidar_stamp_ns, world_frame=grid.frame,
        )
        return False

    def toward_target(self, previous, detection, budget, camera):
        """Follow the image bearing briefly, then stop for a fresh detection."""
        leg = self._begin_leg("directed", previous)
        started = time.monotonic()
        height, width = previous.depth.shape
        left, top, right, bottom = detection["bbox_2d"]
        u = (left + right) * width / 2000
        v = (top + bottom) * height / 2000
        ray = Rotation.from_rotvec([0, camera["pitch"], 0]).apply(optical_ray(previous, u, v))
        q = previous.odom.pose.orientation
        bearing = Rotation.from_quat([q.x, q.y, q.z, q.w]).apply(ray)[:2]
        length = np.linalg.norm(bearing)
        if not np.isfinite(length) or length < 1e-6 or ray[0] <= 0:
            event(logger, "exploration_leg_finished", **leg, reason="invalid_bearing")
            return None

        previous_pose, world_frame = odom_pose(previous.odom)
        seed = self.observe(budget, max_age_s=TRACKING_MAX_AGE_S)
        grid, pose = self._map(seed)
        if grid.frame != world_frame:
            raise ExplorationUnavailable("exploration_frame_changed")
        origin = np.array(pose[:2])
        for distance in (0.60, 0.45, 0.30):
            goal = origin + distance * bearing / length
            if self.planner.path_is_clear(
                grid.grid, origin_xy=grid.origin_xy, resolution=grid.resolution,
                robot_xy=pose[:2], path_xy=goal[None, :],
            ):
                break
        else:
            logger.info("No clear waypoint toward the target bearing")
            event(logger, "exploration_leg_finished", **leg, reason="no_clear_waypoint",
                  grid_stamp_ns=grid.stamp_ns, lidar_stamp_ns=self.lidar_stamp_ns)
            return None
        event(
            logger, "exploration_goal_selected", **leg, goal_xy=goal.tolist(),
            world_frame=world_frame, start_odom=pose, grid_stamp_ns=grid.stamp_ns,
            observation_id=getattr(seed, "observation_id", None),
            lidar_stamp_ns=self.lidar_stamp_ns, bbox_2d=detection["bbox_2d"],
            distance_m=distance,
        )

        deadline = time.monotonic() + self.leg_timeout_s
        tracking = False

        def progress(update):
            nonlocal deadline, tracking
            if update["status"] == "tracking" and not tracking:
                tracking = True
                deadline = time.monotonic() + 2.0

        def leg_budget():
            left = deadline - time.monotonic()
            task_time = budget()
            if left <= 0:
                raise TimeoutError("Target bearing movement window ended")
            return min(task_time, left)

        try:
            self.approach(
                {
                    "goal_xy": tuple(goal),
                    "world_frame": world_frame,
                    "observation": {"stamp_ns": seed.stamp_ns},
                    "timing_ms": {},
                    "exploration_leg": leg,
                },
                leg_budget, self.cancel, progress, accept_path=partial(self._accept_path, leg=leg),
                motion_guard=self.cloud.ready if self.cloud is not None else None,
            )
        except TimeoutError:
            budget()

        if not tracking:
            event(logger, "exploration_leg_finished", **leg, reason="no_accepted_path",
                  elapsed_ms=(time.monotonic() - started) * 1000)
            return None
        self.cancel.wait(min(self.settle_s, budget()))
        stopped_frame = self.observe(
            budget, after_stamp_ns=max(seed.stamp_ns, self.newest_stamp()),
            max_age_s=TRACKING_MAX_AGE_S,
        )
        stopped_pose, frame = odom_pose(stopped_frame.odom)
        if frame != world_frame:
            raise ExplorationUnavailable("exploration_frame_changed")
        event(
            logger, "exploration_leg_finished", **leg, reason="new_view",
            stopped_stamp_ns=stopped_frame.stamp_ns, end_odom=stopped_pose,
            displacement_m=math.dist(previous_pose[:2], stopped_pose[:2]),
            elapsed_ms=(time.monotonic() - started) * 1000,
        )
        return stopped_frame
