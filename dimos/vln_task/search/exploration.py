"""Turn in place to scan nearby headings and approach visible targets with bad depth."""

import logging
import math
import threading
import time
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.plugins.path_tracker.messages import odom_pose
from dimos.plugins.path_tracker.tracker import TURN_TOLERANCE_RAD
from dimos.plugins._agent_common import TaskCancelled
from dimos.plugins.runtime_log import event, recording_enabled
from dimos.vln_task.observation import TRACKING_MAX_AGE_S
from dimos.vln_task.perception import optical_ray
from dimos.plugins.xnavdp.client import stamp_ns

from .search import ExplorationUnavailable

logger = logging.getLogger('dimos.plugins.active_search.exploration')
_DIRECTED_STEP_M = 2.0
_DIRECTED_MOTION_WINDOW_S = 6.0


def observe_stopped(observe, newest_stamp, previous_stamp, remaining, settle_s, **options):
    """Wait for samples received after the stop and settling window."""
    settled_at = time.monotonic() + settle_s
    after_stamp = max(previous_stamp, newest_stamp())
    return observe(
        remaining, after_stamp_ns=after_stamp, received_after=settled_at,
        max_age_s=TRACKING_MAX_AGE_S, **options,
    )


def _log_leg_error(leg, phase, exc, started, **fields):
    if started is None:
        return
    if isinstance(exc, ExplorationUnavailable):
        reason = str(exc)
    elif isinstance(exc, TaskCancelled):
        reason = "task_cancelled"
    elif isinstance(exc, TimeoutError):
        reason = "no_new_view" if phase == "stopped_observation" else "timeout"
    else:
        reason = type(exc).__name__
    event(
        logger, "exploration_leg_finished", **leg, reason=reason, phase=phase,
        error_type=type(exc).__name__, elapsed_ms=(time.monotonic() - started) * 1000, **fields,
    )


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
        if age > 0.75:
            event(
                logger, "exploration_lidar_rejected", reason="stale",
                lidar_stamp_ns=stamp, odom_stamp_ns=odom_stamp,
                age_ms=age * 1000, gap_ms=gap / 1e6,
            )
            return None
        if gap > 750_000_000:
            logger.warning(
                "Exploration LiDAR/odom timestamp gap: %.1f ms; using latest world-frame cloud",
                gap / 1e6,
            )
            event(logger, "exploration_lidar_gap", lidar_stamp_ns=stamp,
                  odom_stamp_ns=odom_stamp, age_ms=age * 1000, gap_ms=gap / 1e6)
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
    """Sample four headings spaced 75 degrees from the first observation."""

    def __init__(
        self, observe, newest_stamp, turn, first, cancel, *, settle_s=0.15,
        heading_offsets=range(75, 360, 75), leg=None,
    ):
        self.observe = observe
        self.newest_stamp = newest_stamp
        self.turn = turn
        pose, self.world_frame = odom_pose(first.odom)
        self.initial_yaw = pose[2]
        self.headings = enumerate((
            math.remainder(self.initial_yaw + math.radians(angle), math.tau)
            for angle in heading_offsets
        ), start=1)
        self.cancel = cancel
        self.settle_s = settle_s
        self.leg = leg or {"leg_kind": "rotation"}

    def __call__(self, previous, remaining, on_progress):
        heading = next(self.headings, None)
        if heading is None:
            return None
        step, yaw = heading
        started = time.monotonic() if recording_enabled() else None
        leg = {
            **self.leg, "source_stamp_ns": previous.stamp_ns,
            "sweep_step": step, "goal_yaw": yaw, "world_frame": self.world_frame,
        }
        event(logger, "exploration_leg_started", **leg,
              observation_id=getattr(previous, "observation_id", None))

        def accept_odom(odom):
            pose, frame = odom_pose(odom)
            return frame == self.world_frame and abs(
                math.remainder(yaw - pose[2], math.tau)
            ) <= TURN_TOLERANCE_RAD

        phase = "turn"
        try:
            reason = self.turn(yaw, self.world_frame, remaining)
            remaining()
            if reason is not None:
                raise ExplorationUnavailable(reason)
            phase = "stopped_observation"
            stopped = observe_stopped(
                self.observe, self.newest_stamp, previous.stamp_ns, remaining, self.settle_s,
                accept_odom=accept_odom,
            )
        except Exception as exc:
            _log_leg_error(leg, phase, exc, started)
            raise
        if started is not None:
            pose, _ = odom_pose(stopped.odom)
            event(
                logger, "exploration_leg_finished", **leg, reason="new_view", phase=phase,
                stopped_stamp_ns=stopped.stamp_ns, observation_id=getattr(stopped, "observation_id", None),
                end_odom=pose, heading_error_rad=math.remainder(yaw - pose[2], math.tau),
                elapsed_ms=(time.monotonic() - started) * 1000,
            )
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

    def toward_target(self, previous, detection, budget, camera):
        """Follow the image bearing briefly, then stop for a fresh detection."""
        leg = self._begin_leg("directed", previous)
        started = time.monotonic() if recording_enabled() else None
        try:
            selected = self._bearing_goal(previous, detection, budget, camera, leg)
        except Exception as exc:
            _log_leg_error(leg, "bearing_selection", exc, started)
            raise
        if selected is None:
            return None
        goal, seed, previous_pose, world_frame = selected

        deadline = time.monotonic() + self.leg_timeout_s
        tracking = False

        def progress(update):
            nonlocal deadline, tracking
            if update["status"] == "tracking" and not tracking:
                tracking = True
                deadline = time.monotonic() + _DIRECTED_MOTION_WINDOW_S

        def leg_budget():
            left = deadline - time.monotonic()
            task_time = budget()
            if left <= 0:
                raise TimeoutError("Target bearing movement window ended")
            return min(task_time, left)

        motion_reason = None
        try:
            try:
                result = self.approach(
                    {
                        "goal_xy": tuple(goal),
                        "world_frame": world_frame,
                        "observation": {"stamp_ns": seed.stamp_ns},
                        "timing_ms": {},
                        "exploration_leg": leg,
                    },
                    leg_budget, self.cancel, progress,
                    motion_guard=self.cloud.ready if self.cloud is not None else None,
                )
                if started is not None and isinstance(result, dict):
                    motion_reason = result.get("status")
            except TimeoutError:
                budget()
                if started is not None:
                    motion_reason = "motion_window_elapsed" if time.monotonic() >= deadline else "motion_timeout"
        except Exception as exc:
            _log_leg_error(leg, "motion", exc, started, tracking_started=tracking)
            raise

        if not tracking:
            if started is not None:
                event(logger, "exploration_leg_finished", **leg, reason="no_accepted_path",
                      phase="motion", motion_reason=motion_reason,
                      elapsed_ms=(time.monotonic() - started) * 1000)
            return None
        try:
            stopped_frame = observe_stopped(
                self.observe, self.newest_stamp, seed.stamp_ns, budget, self.settle_s,
            )
            stopped_pose, frame = odom_pose(stopped_frame.odom)
            if frame != world_frame:
                raise ExplorationUnavailable("exploration_frame_changed")
        except Exception as exc:
            _log_leg_error(leg, "stopped_observation", exc, started, motion_reason=motion_reason)
            raise
        if started is not None:
            event(
                logger, "exploration_leg_finished", **leg, reason="new_view",
                phase="stopped_observation", motion_reason=motion_reason,
                stopped_stamp_ns=stopped_frame.stamp_ns, end_odom=stopped_pose,
                observation_id=getattr(stopped_frame, "observation_id", None),
                displacement_m=math.dist(previous_pose[:2], stopped_pose[:2]),
                elapsed_ms=(time.monotonic() - started) * 1000,
            )
        return stopped_frame

    def _bearing_goal(self, previous, detection, budget, camera, leg):
        height, width = previous.depth.shape
        left, top, right, bottom = detection["bbox_2d"]
        u = (left + right) * width / 2000
        v = (top + bottom) * height / 2000
        ray = Rotation.from_rotvec([0, camera["pitch"], 0]).apply(optical_ray(previous, u, v))
        q = previous.odom.pose.orientation
        bearing = Rotation.from_quat([q.x, q.y, q.z, q.w]).apply(ray)[:2]
        length = np.linalg.norm(bearing)
        if not np.isfinite(length) or length < 1e-6 or ray[0] <= 0:
            event(logger, "exploration_leg_finished", **leg, reason="invalid_bearing",
                  phase="bearing_selection", bbox_2d=detection["bbox_2d"])
            return None

        previous_pose, world_frame = odom_pose(previous.odom)
        seed = self.observe(budget, max_age_s=TRACKING_MAX_AGE_S)
        pose, seed_frame = odom_pose(seed.odom)
        if seed_frame != world_frame:
            raise ExplorationUnavailable("exploration_frame_changed")
        goal = np.array(pose[:2]) + _DIRECTED_STEP_M * bearing / length
        event(
            logger, "exploration_goal_selected", **leg, goal_xy=goal.tolist(),
            world_frame=world_frame, start_odom=pose, source_stamp_ns=seed.stamp_ns,
            observation_id=getattr(seed, "observation_id", None),
            bbox_2d=detection["bbox_2d"], distance_m=_DIRECTED_STEP_M,
        )

        return goal, seed, previous_pose, world_frame
