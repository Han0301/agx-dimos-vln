# Based on dimos_simple/dimos/navigation/xnavdp/client.py (Apache-2.0).
"""Native point-goal client: original topics, independent executor, no relays."""

from __future__ import annotations

import copy
import json
import logging
import math
import os
import threading
import time

from dimos.plugins.runtime_log import diagnostic_artifact, event, recording_enabled

logger = logging.getLogger(__name__)

VERSION = "xnavdp-ros2-topics-v3"
EXPECTED_RELEASE = "unified-20261104"

RGB_TOPIC = "/camera/camera/color/image_raw/compressed"
ODOM_TOPIC = "/odom"
GOAL_TOPIC = "/xnavdp/point_goal"
_INLINE_REPLAY_BYTES = 64 * 1024
_MAX_NATIVE_REPLAY_POINTS = 4096


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def relative_goal_xy(position, quaternion, goal):
    """Convert a planar goal from robot-local to world coordinates."""
    values = [*position, *quaternion, *goal]
    if len(position) != 2 or len(quaternion) != 4 or len(goal) != 2:
        raise ValueError("expected XY position, XYZW quaternion and XY local goal")
    if not all(math.isfinite(v) for v in values):
        raise ValueError("non-finite goal or odometry")
    x, y, z, w = quaternion
    if sum(v * v for v in quaternion) < 1e-12:
        raise ValueError("zero odometry quaternion")
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    c, s = math.cos(yaw), math.sin(yaw)
    return position[0] + c * goal[0] - s * goal[1], position[1] + s * goal[
        0
    ] + c * goal[1]


def build_goal(rgb, odom, local_xy):
    from geometry_msgs.msg import PoseStamped

    if not odom.header.frame_id or stamp_ns(rgb.header) <= 0:
        raise ValueError("missing odom frame or RGB capture timestamp")
    p, q = odom.pose.position, odom.pose.orientation
    x, y = relative_goal_xy((p.x, p.y), (q.x, q.y, q.z, q.w), local_xy)
    goal = PoseStamped()
    # Preserve integer nanoseconds; no float round-trip through UNIX seconds.
    goal.header.stamp = copy.deepcopy(rgb.header.stamp)
    goal.header.frame_id = odom.header.frame_id
    goal.pose.position.x, goal.pose.position.y = x, y
    goal.pose.orientation.w = 1.0
    return goal


def validate_result(parts, goal):
    """Validate native base_link Path + JSON result and exact observation time."""
    result = json.loads(parts["result"].data)
    expected = stamp_ns(goal.header)
    if (
        type(result.get("request_stamp_ns")) is not int
        or result["request_stamp_ns"] != expected
    ):
        raise ValueError("result: mismatched request timestamp")
    if result.get("status") != "success":
        raise ValueError(result.get("error", "inference failed"))
    path = parts["path"]
    frame = result.get("trajectory_frame")
    if frame != "base_link" or result.get("goal_frame") != frame:
        raise ValueError("trajectory and local goal must use base_link")
    if path.header.frame_id != frame or stamp_ns(path.header) != expected:
        raise ValueError("path header differs from the correlated result")
    if result.get("world_frame") != goal.header.frame_id:
        raise ValueError("result reference world differs from goal")

    def vector(value, length):
        if (
            not isinstance(value, list)
            or len(value) != length
            or any(
                isinstance(v, bool)
                or not isinstance(v, (int, float))
                or not math.isfinite(v)
                for v in value
            )
        ):
            raise ValueError("invalid model vector")
        return value

    selected = result["trajectory"]
    candidates, scores = result["all_trajectory"], result["all_values"]
    if len(selected) != 1 or not selected[0] or len(path.poses) != len(selected[0]):
        raise ValueError("invalid selected trajectory length")
    for pose, point in zip(path.poses, selected[0]):
        vector(point, 3)
        if pose.header.frame_id != frame or stamp_ns(pose.header) != expected:
            raise ValueError("path point header differs from request")
        if any(
            not math.isclose(
                getattr(pose.pose.position, axis), value, rel_tol=1e-9, abs_tol=1e-6
            )
            for axis, value in zip("xyz", point)
        ):
            raise ValueError("Path and JSON trajectory differ")
    if (
        len(candidates) != 1
        or not candidates[0]
        or len(scores) != 1
        or len(scores[0]) != len(candidates[0])
    ):
        raise ValueError("invalid candidate/score count")
    vector(scores[0], len(candidates[0]))
    for candidate in candidates[0]:
        if len(candidate) != len(selected[0]):
            raise ValueError("invalid candidate length")
        for point in candidate:
            vector(point, 3)
    position = vector(result["robot_pos"], 3)
    quaternion = vector(result["robot_quat"], 4)
    local_goal = vector(result["goal"], 2)
    world_goal = relative_goal_xy(position[:2], quaternion, local_goal)
    if any(
        not math.isclose(v, getattr(goal.pose.position, axis), abs_tol=1e-6)
        for axis, v in zip("xy", world_goal)
    ):
        raise ValueError("local goal and reference pose do not reconstruct world goal")
    return result


def _record_trajectory(parts, result, request):
    path = parts["path"]
    if len(path.poses) > _MAX_NATIVE_REPLAY_POINTS:
        artifact = diagnostic_artifact(
            f"xnavdp-{request}.json", parts["result"].data, request_stamp_ns=request,
        )
        event(
            logger, "xnavdp_trajectory", request_stamp_ns=request, artifact_path=artifact,
            native_path_omitted="point_limit",
        )
        return

    poses = []
    for item in path.poses:
        p = item.pose.position
        q = getattr(item.pose, "orientation", None)
        orientation = [q.x, q.y, q.z, q.w] if q is not None else None
        poses.append({
            "position_xyz": [p.x, p.y, p.z],
            "orientation_xyzw": (
                [value if math.isfinite(value) else None for value in orientation]
                if orientation is not None else None
            ),
        })
    replay = {
        "native_path": {
            "frame_id": path.header.frame_id, "stamp_ns": stamp_ns(path.header), "poses": poses,
        },
        "result": {name: result[name] for name in (
            "request_stamp_ns", "trajectory", "all_trajectory", "all_values", "trajectory_frame",
            "goal_frame", "world_frame", "robot_pos", "robot_quat", "goal",
        )},
    }
    if len(parts["result"].data) <= _INLINE_REPLAY_BYTES and len(poses) <= 512:
        event(logger, "xnavdp_trajectory", request_stamp_ns=request, **replay)
    else:
        artifact = diagnostic_artifact(
            f"xnavdp-{request}.json", json.dumps(replay, separators=(",", ":")),
            request_stamp_ns=request,
        )
        event(
            logger, "xnavdp_trajectory", request_stamp_ns=request, artifact_path=artifact,
            artifact_outcome="queued" if artifact else "unavailable",
        )


class XNavDPBusy(RuntimeError):
    pass


class XNavDPInferenceError(RuntimeError):
    pass


class XNavDPClient:
    def __init__(self, settings, *, node_name="xnavdp_comm_test"):
        self.settings = settings
        self.node_name = node_name
        self.node = self.context = self.executor = self.thread = None
        self.goal_pub = None
        self.closed = False
        self.failure = None
        self.lock = threading.Lock()
        self.request_lock = threading.Lock()
        self.event = threading.Event()
        self.pending = None
        self.parts = {}
        self.part_arrivals = None
        self.rejected_parts = None
        self.error = None
        self.latest_rgb = self.latest_odom = None
        self.subscriptions = None
        self.last_rgb_at = self.last_odom_at = 0.0
        self.last_request_stamp = None

    def start(self):
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.signals import SignalHandlerOptions
        from std_srvs.srv import Trigger

        self.rclpy, self.pose_type, self.trigger_type = rclpy, PoseStamped, Trigger
        self.context = Context()
        try:
            rclpy.init(
                context=self.context,
                domain_id=int(os.environ.get("ROS_DOMAIN_ID", "0")),
                signal_handler_options=SignalHandlerOptions.NO,
            )
            self.node = Node(self.node_name, context=self.context)
            self.health_client = self.node.create_client(Trigger, "/xnavdp/health")
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)

            def spin():
                try:
                    self.executor.spin()
                except Exception as exc:
                    if not self.closed:
                        logger.error(
                            "ROS executor failed error_type=%s", type(exc).__name__
                        )
                        self.failure = exc
                        self.event.set()

            self.thread = threading.Thread(target=spin, daemon=True)
            self.thread.start()
            logger.info("ROS client started node=%s", self.node_name)
            return self
        except Exception:
            self.stop()
            raise

    def _rgb(self, msg):
        with self.lock:
            self.latest_rgb, self.last_rgb_at = msg, time.monotonic()

    def _odom(self, msg):
        with self.lock:
            self.latest_odom, self.last_odom_at = msg, time.monotonic()

    def _receive(self, key, msg, header):
        with self.lock:
            if self.pending is None:
                return
            if stamp_ns(header) != self.pending:
                if self.rejected_parts is not None:
                    self.rejected_parts["wrong_stamp"] += 1
                return
            self.parts[key] = msg
            if self.part_arrivals is not None:
                self.part_arrivals.setdefault(key, time.monotonic())
            if all(k in self.parts for k in ("path", "result")):
                self.event.set()

    def _result(self, msg):
        try:
            result = json.loads(msg.data)
        except (ValueError, TypeError):
            result = None
        with self.lock:
            if self.pending is None:
                return
            if (
                not isinstance(result, dict)
                or type(result.get("request_stamp_ns")) is not int
            ):
                if self.rejected_parts is not None:
                    self.rejected_parts["malformed_result"] += 1
                return
            if result["request_stamp_ns"] != self.pending:
                if self.rejected_parts is not None:
                    self.rejected_parts["wrong_stamp"] += 1
                return
            if self.part_arrivals is not None:
                self.part_arrivals.setdefault("result", time.monotonic())
            if result.get("status") != "success":
                self.error = str(result.get("error", "inference failed"))
                self.event.set()
            self.parts["result"] = msg
            if "path" in self.parts:
                self.event.set()

    def _check(self):
        if self.closed:
            raise RuntimeError("XNavDP client is closed")
        if self.failure:
            raise RuntimeError(f"ROS receiver failed: {self.failure}")

    def _service(self, client, timeout, *, check=None):
        self._check()
        if check is not None:
            check()
        deadline = time.monotonic() + timeout
        while not client.wait_for_service(
            timeout_sec=min(0.2, max(0.0, deadline - time.monotonic()))
        ):
            self._check()
            if check is not None:
                check()
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"{client.srv_name} unavailable; check existing DDS 7411 configuration"
                )
        if check is not None:
            check()
        future = client.call_async(self.trigger_type.Request())
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        try:
            while not done.wait(0.1):
                self._check()
                if check is not None:
                    check()
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"{client.srv_name} timed out")
            if check is not None:
                check()
            response = future.result()
            if response is None or not response.success:
                raise RuntimeError(
                    response.message if response else str(future.exception())
                )
            return response.message
        finally:
            if not future.done():
                client.remove_pending_request(future)

    def health(self, *, check=None):
        data = json.loads(
            self._service(self.health_client, self.settings["service_timeout_s"], check=check)
        )
        if data.get("transport") != VERSION or data.get("release") != EXPECTED_RELEASE:
            raise RuntimeError(
                "XNavDP protocol/release differs from trajectory/result schema; inspect /xnavdp/health"
            )
        return data

    def prepare(self, reset_if_needed=False, *, check=None):
        deadline = time.monotonic() + self.settings["sensor_timeout_s"]
        while True:
            health = self.health() if check is None else self.health(check=check)
            if health.get("busy"):
                raise XNavDPBusy(
                    "XNavDP is busy; keep the active task running and test later"
                )
            missing = any(
                health.get("buffers", {}).get(k, 0) <= 0
                for k in ("rgb", "depth", "odom")
            )
            if not missing and health.get("calibrated"):
                break
            if time.monotonic() >= deadline:
                if missing:
                    raise RuntimeError(
                        "XNavDP has missing RGB/depth/odom inputs; no goal sent"
                    )
                raise RuntimeError(
                    "Server has no camera calibration; no duplicate CameraInfo will be published"
                )
            self._check()
            time.sleep(min(0.2, max(0, deadline - time.monotonic())))
        if not health.get("initialized"):
            if not reset_if_needed:
                raise RuntimeError(
                    "Model is not initialized; reset_if_needed is false, no reset/goal sent"
                )
            if self.node.count_publishers(GOAL_TOPIC):
                raise RuntimeError(
                    "Another point-goal publisher exists; refusing model reset"
                )
            reset = self.node.create_client(self.trigger_type, "/xnavdp/reset")
            try:
                self._service(reset, 180.0)
            finally:
                self.node.destroy_client(reset)
            health = self.health()
            if not health.get("initialized"):
                raise RuntimeError("Model is still uninitialized")
        return health

    def next_goal(self, local_xy):
        deadline = time.monotonic() + self.settings["sensor_timeout_s"]
        while time.monotonic() < deadline:
            self._check()
            with self.lock:
                rgb, odom = self.latest_rgb, self.latest_odom
                now = time.monotonic()
                fresh = now - self.last_rgb_at <= 1.0 and now - self.last_odom_at <= 1.0
                if (
                    fresh
                    and rgb is not None
                    and odom is not None
                    and stamp_ns(rgb.header) != self.last_request_stamp
                ):
                    return build_goal(rgb, odom, local_xy)
            time.sleep(0.02)
        raise TimeoutError("No fresh original RGB/odom for a new request")

    def _subscribe_for_inference(self):
        if self.subscriptions is not None:
            return
        from nav_msgs.msg import Path
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CompressedImage
        from std_msgs.msg import String

        PoseStamped = self.pose_type
        self.subscriptions = [
            self.node.create_subscription(
                CompressedImage, RGB_TOPIC, self._rgb, qos_profile_sensor_data
            ),
            self.node.create_subscription(
                PoseStamped, ODOM_TOPIC, self._odom, qos_profile_sensor_data
            ),
            self.node.create_subscription(
                Path,
                "/xnavdp/trajectory",
                lambda m: self._receive("path", m, m.header),
                10,
            ),
            self.node.create_subscription(String, "/xnavdp/result", self._result, 10),
        ]

    def warmup_requests(self):
        """Start DDS discovery without waiting, sending a goal, or resetting."""
        self._check()
        self._subscribe_for_inference()
        if self.goal_pub is None:
            if self.node.count_publishers(GOAL_TOPIC):
                return False
            self.goal_pub = self.node.create_publisher(self.pose_type, GOAL_TOPIC, 5)
        return True

    def open_requests(self, *, check=None):
        if not self.warmup_requests():
            raise RuntimeError(
                "Another point-goal publisher exists; refusing active test"
            )
        deadline = time.monotonic() + self.settings["sensor_timeout_s"]
        while self.goal_pub.get_subscription_count() == 0:
            self._check()
            if check is not None:
                check()
            if time.monotonic() >= deadline:
                raise TimeoutError("No subscriber on /xnavdp/point_goal")
            time.sleep(0.05)
        if self.node.count_publishers(GOAL_TOPIC) != 1:
            raise RuntimeError(
                "Multiple point-goal publishers; refusing concurrent test"
            )

    def infer(self, goal, *, check=None, timing_ms=None):
        """Return the original Path and JSON String messages."""
        with self.request_lock:
            request = stamp_ns(goal.header)
            recording = recording_enabled()
            if recording:
                position = goal.pose.position
                fields = {
                    "request_stamp_ns": request,
                    "goal_xy": [position.x, position.y],
                    "world_frame": goal.header.frame_id,
                }
                event(logger, "xnavdp_request_started", **fields)
            timings = timing_ms
            if recording and timings is None:
                timings = {}
            started = phase_at = time.monotonic() if timings is not None else None
            stage = "discovery"
            opened = False
            sent_at = None
            try:
                self._check()
                if check is not None:
                    check()
                self.open_requests(check=check)
                if timings is not None:
                    prepared = time.monotonic()
                    timings["xnavdp_discovery"] = (prepared - phase_at) * 1000
                    phase_at = prepared
                stage = "health"
                # Refresh readiness for each request; never reset here.
                health = self.prepare(False, check=check)
                if recording and isinstance(health, dict):
                    fields["server_release"] = health.get("release")
                    fields["server_transport"] = health.get("transport")
                if timings is not None:
                    health_at = time.monotonic()
                    timings["xnavdp_health"] = (health_at - phase_at) * 1000
                    phase_at = health_at
                stage = "publish"
                if check is not None:
                    check()
                if request == self.last_request_stamp:
                    raise ValueError("RGB timestamp already used")
                with self.lock:
                    self.pending, self.parts, self.error = request, {}, None
                    self.part_arrivals = {} if recording else None
                    self.rejected_parts = {"wrong_stamp": 0, "malformed_result": 0} if recording else None
                    self.last_request_stamp = request
                    self.event.clear()
                    opened = True
                sent_at = time.monotonic()
                if check is not None:
                    check()
                self.goal_pub.publish(goal)
                logger.info("request published stamp_ns=%d", request)
                stage, phase_at = "response", sent_at
                deadline = sent_at + self.settings["response_timeout_s"]
                while not self.event.wait(0.1):
                    self._check()
                    if check is not None:
                        check()
                    if time.monotonic() >= deadline:
                        with self.lock:
                            missing = sorted({"path", "result"} - self.parts.keys())
                        logger.warning(
                            "request timed out stamp_ns=%d missing=%s",
                            request,
                            ",".join(missing),
                        )
                        raise TimeoutError(f"XNavDP result timeout; missing {missing}")
                self._check()
                with self.lock:
                    if self.error:
                        logger.warning(
                            "request rejected by server stamp_ns=%d", request
                        )
                        raise XNavDPInferenceError(self.error)
                    parts = dict(self.parts)
                if timings is not None:
                    received = time.monotonic()
                    timings["xnavdp_response"] = (received - sent_at) * 1000
                    phase_at = received
                stage = "validation"
                result = validate_result(parts, goal)
                if timings is not None:
                    timings["xnavdp_validation"] = (time.monotonic() - received) * 1000
                if check is not None:
                    check()
                elapsed_s = time.monotonic() - sent_at
                path_points = len(parts["path"].poses)
                logger.info(
                    "request completed stamp_ns=%d path_points=%d elapsed_ms=%.1f",
                    request, path_points, elapsed_s * 1000,
                )
                if recording:
                    fields.update(
                        outcome="success", path_points=path_points,
                        local_goal_xy=result.get("goal"), trajectory_frame=result.get("trajectory_frame"),
                        server_odom_xyz=result.get("robot_pos"),
                        server_depth_gap_ms=result.get("depth_rgb_gap_ms"),
                        server_odom_gap_ms=result.get("odom_rgb_gap_ms"),
                    )
                    _record_trajectory(parts, result, request)
                return parts, elapsed_s
            except Exception as exc:
                if recording:
                    fields.update(
                        outcome="failed",
                        error_type=type(exc).__name__,
                        reason=str(exc)[:300],
                        failed_stage_ms=(time.monotonic() - phase_at) * 1000,
                    )
                raise
            finally:
                if opened:
                    with self.lock:
                        if recording:
                            fields["missing_parts"] = sorted({"path", "result"} - self.parts.keys())
                            fields["rejected_parts"] = dict(self.rejected_parts)
                            if sent_at is not None:
                                fields["part_ms"] = {
                                    part: (arrived - sent_at) * 1000
                                    for part, arrived in self.part_arrivals.items()
                                }
                        self.pending = None
                        self.parts = {}
                if recording:
                    event(
                        logger, "xnavdp_request_finished", **fields, stage=stage,
                        elapsed_ms=(time.monotonic() - started) * 1000, timing_ms=dict(timings),
                    )

    def stop(self):
        if self.closed:
            return
        self.closed = True
        self.event.set()
        if self.executor is not None:
            self.executor.shutdown(timeout_sec=2)
        if self.thread is not None:
            self.thread.join(timeout=2)
        if self.node is not None:
            self.node.destroy_node()
        if self.context is not None and self.context.ok():
            self.context.try_shutdown()
