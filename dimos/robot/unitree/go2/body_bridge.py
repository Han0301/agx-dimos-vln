# Copyright 2025-2026 Dimensional Inc. Licensed under the Apache License, Version 2.0.
"""Native ROS body bridge with the dimos_simple topic types and TF frames."""

import json
import logging
import math
import os
import threading
import time

from dimos.plugins.runtime_log import event, recording_enabled

log = logging.getLogger(__name__)
STOP_WINDOW_S = 2.0
STOP_LINEAR_MPS = 0.03
STOP_ANGULAR_RPS = 0.08
STOP_LOW_MOTION_SAMPLES = 2


def read_aes_key(env_name):
    """Read device credentials without including their value in logs or errors."""
    value = os.environ.get(env_name, "").strip()
    if not value:
        raise RuntimeError(
            f"Missing {env_name}; load ~/.config/dimos/unitree.env before connecting"
        )
    if len(value) != 32 or any(c not in "0123456789abcdefABCDEF" for c in value):
        raise RuntimeError(f"{env_name} must contain exactly 32 hexadecimal characters")
    return value


def bounded_command(values, linear_limit, angular_limit):
    if not all(math.isfinite(v) for v in values):
        raise ValueError("non-finite cmd_vel")
    x, y, w = values
    return (
        max(-linear_limit, min(linear_limit, x)),
        max(-linear_limit, min(linear_limit, y)),
        max(-angular_limit, min(angular_limit, w)),
    )


class Go2BodyBridge:
    def __init__(self, node, config):
        self.node = node
        self.config = config
        self.connection = None
        self.last_command = None
        self.commands_active = False
        self.zero_forwarded_at = None
        self.stop_samples = None
        self.stop_lock = threading.Lock()
        self.last_odom_sample = None
        self.timers = []

    def start(self):
        from geometry_msgs.msg import PoseStamped, Twist
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from tf2_msgs.msg import TFMessage
        from unitree_webrtc_connect.constants import RTC_TOPIC

        from dimos.robot.unitree.connection import UnitreeWebRTCConnection

        b = self.config["body"]
        topics = self.config["topics"]
        aes_key = read_aes_key(b["aes_key_env"])
        os.environ["DIMOS_GO2_WIRED_INTERFACE"] = b["interface"]
        self.pose_type = PoseStamped
        self.odom = self.node.create_publisher(
            PoseStamped, topics["odom"], qos_profile_sensor_data
        )
        self.lidar = self.node.create_publisher(PointCloud2, topics["lidar"], 10)
        self.lowstate = self.node.create_publisher(
            String, topics["lowstate"], qos_profile_sensor_data
        )
        self.tf = self.node.create_publisher(TFMessage, topics["tf"], 10)
        self.connection = UnitreeWebRTCConnection(
            b["ip"], mode=b["mode"], aes_128_key=aes_key
        )
        self.connection.start()
        self.connection.subscribe(RTC_TOPIC["ROBOTODOM"], self._on_odom)
        self.connection.subscribe(RTC_TOPIC["ULIDAR_ARRAY"], self._on_lidar)
        self.connection.subscribe(RTC_TOPIC["LOW_STATE"], self._on_lowstate)
        self.subscription = self.node.create_subscription(
            Twist, topics["cmd_vel"], self._on_command, 10
        )
        self.services = []
        for name in ("standup", "liedown", "balance_stand"):
            self.services.append(
                self.node.create_service(
                    Trigger, "/go2/" + name, self._posture_callback(name)
                )
            )
        self.timers.append(self.node.create_timer(0.05, self._watchdog))
        log.info("Go2 connected; allow_motion=%s", b["allow_motion"])

    def _posture_callback(self, name):
        def callback(request, response):
            if not self.config["body"]["allow_motion"]:
                response.success = False
                response.message = "body.allow_motion is false"
                return response
            try:
                response.success = self.connection.posture(name)
                response.message = (
                    "completed" if response.success else "robot rejected command"
                )
            except Exception as exc:
                response.success = False
                response.message = str(exc)
            return response

        return callback

    def _on_command(self, msg):
        if not self.config["body"]["allow_motion"]:
            return
        received_at = time.monotonic()
        requested = (msg.linear.x, msg.linear.y, msg.angular.z)
        b = self.config["body"]
        try:
            values = bounded_command(requested, b["max_linear_mps"], b["max_angular_rps"])
        except ValueError:
            self._zero("invalid_cmd_vel")
            log.warning("rejected non-finite cmd_vel")
            return
        active = any(values)
        first_command = self.last_command is None
        transition = active != self.commands_active
        try:
            self.connection.move(*values)
        except Exception as exc:
            event(
                log, "body_command_failed", source="cmd_vel", received_velocity=requested,
                forwarded_velocity=values, error_type=type(exc).__name__,
                elapsed_ms=(time.monotonic() - received_at) * 1000,
            )
            raise
        self.last_command = time.monotonic()
        if transition:
            log.info(
                "cmd_vel forwarded to Go2 active=%s linear_x=%.3f linear_y=%.3f angular_z=%.3f",
                active, *values,
            )
        if first_command or transition:
            if recording_enabled():
                with self.stop_lock:
                    self.zero_forwarded_at = None if active else self.last_command
                    self.stop_samples = 0 if transition and not active else None
            event(
                log, "body_command_forwarded", source="cmd_vel", active=active,
                received_velocity=requested, forwarded_velocity=values,
                received_monotonic_ns=int(received_at * 1e9),
                forwarded_monotonic_ns=int(self.last_command * 1e9),
                forwarding_ms=(self.last_command - received_at) * 1000,
            )
        self.commands_active = active

    def _zero(self, reason):
        was_active = self.commands_active
        self.commands_active = False
        if self.connection is not None:
            started = time.monotonic()
            self.connection.move(0.0, 0.0, 0.0)
            forwarded_at = time.monotonic()
            if was_active and recording_enabled():
                with self.stop_lock:
                    self.zero_forwarded_at = forwarded_at
                    self.stop_samples = 0
            event(
                log, "body_command_forwarded", source=reason, active=False,
                forwarded_velocity=(0.0, 0.0, 0.0),
                forwarded_monotonic_ns=int(forwarded_at * 1e9),
                forwarding_ms=(forwarded_at - started) * 1000,
            )

    def _watchdog(self):
        if (
            self.commands_active
            and time.monotonic() - self.last_command
            > self.config["body"]["command_timeout_s"]
        ):
            self._zero("watchdog")
            log.info("cmd_vel watchdog forwarded zero to Go2")
        if not recording_enabled():
            return
        with self.stop_lock:
            zero_at = self.zero_forwarded_at
            expired = (
                self.stop_samples is not None and zero_at is not None
                and time.monotonic() - zero_at > STOP_WINDOW_S
            )
            if expired:
                self.stop_samples = None
        if expired:
            event(log, "body_stop_observation_expired", source="odom_pose_delta",
                  zero_forwarded_monotonic_ns=int(zero_at * 1e9), window_ms=STOP_WINDOW_S * 1000)

    def _sample_motion(self, pose, values):
        if not recording_enabled() or self.last_command is None:
            return
        now = time.monotonic()
        zero_at = self.zero_forwarded_at
        if not self.commands_active and (zero_at is None or now - zero_at > STOP_WINDOW_S):
            self.last_odom_sample = None
            return
        last = self.last_odom_sample
        if last is not None and now - last[0] < 0.2:
            return
        x, y, _, qx, qy, qz, qw = values
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        self.last_odom_sample = now, x, y, yaw
        if last is None or now - last[0] > 1.0:
            with self.stop_lock:
                if self.stop_samples is not None:
                    self.stop_samples = 0
            return
        elapsed = now - last[0]
        speed = math.hypot(x - last[1], y - last[2]) / elapsed
        yaw_rate = math.remainder(yaw - last[3], 2 * math.pi) / elapsed
        stamp = pose.header.stamp
        fields = {
            "source": "odom_pose_delta",
            "odom_stamp_ns": stamp.sec * 1_000_000_000 + stamp.nanosec,
            "world_frame": pose.header.frame_id,
            "odom_xy": (x, y),
            "speed_mps_estimate": speed,
            "yaw_rate_rps_estimate": yaw_rate,
        }
        event(
            log, "body_motion_sample", **fields, yaw_rad=yaw,
            sample_interval_ms=elapsed * 1000, command_active=self.commands_active,
            since_zero_ms=(now - zero_at) * 1000 if zero_at is not None else None,
        )
        with self.stop_lock:
            if (
                self.stop_samples is None or zero_at is None
                or zero_at != self.zero_forwarded_at or last[0] < zero_at
            ):
                return
            low_motion = speed <= STOP_LINEAR_MPS and abs(yaw_rate) <= STOP_ANGULAR_RPS
            self.stop_samples = self.stop_samples + 1 if low_motion else 0
            observed = self.stop_samples == STOP_LOW_MOTION_SAMPLES
            if observed:
                self.stop_samples = None
        if observed:
            event(
                log, "body_stop_observed", **fields,
                zero_forwarded_monotonic_ns=int(zero_at * 1e9),
                elapsed_ms=(now - zero_at) * 1000, window_ms=STOP_WINDOW_S * 1000,
                consecutive_low_motion_samples=STOP_LOW_MOTION_SAMPLES,
                linear_threshold_mps=STOP_LINEAR_MPS, angular_threshold_rps=STOP_ANGULAR_RPS,
            )

    def _on_odom(self, msg):
        try:
            from geometry_msgs.msg import TransformStamped
            from tf2_msgs.msg import TFMessage

            pose = self.pose_type()
            pose.header.stamp = self.node.get_clock().now().to_msg()
            pose.header.frame_id = self.config["body"]["world_frame"]
            raw = msg["data"]["pose"]
            for axis in "xyz":
                setattr(pose.pose.position, axis, float(raw["position"][axis]))
            for axis in "xyzw":
                setattr(pose.pose.orientation, axis, float(raw["orientation"][axis]))
            values = [getattr(pose.pose.position, a) for a in "xyz"] + [
                getattr(pose.pose.orientation, a) for a in "xyzw"
            ]
            if not all(math.isfinite(v) for v in values):
                raise ValueError("invalid odom")
            self.odom.publish(pose)
            self._sample_motion(pose, values)
            b = self.config["body"]
            c = self.config["camera"]
            definitions = [
                (b["world_frame"], b["base_frame"], values[:3], values[3:]),
                (
                    b["base_frame"],
                    c["mount_frame"],
                    c["translation"],
                    [0.0, math.sin(c["pitch"] / 2), 0.0, math.cos(c["pitch"] / 2)],
                ),
            ]
            transforms = []
            for parent, child, translation, rotation in definitions:
                tf = TransformStamped()
                tf.header.stamp = pose.header.stamp
                tf.header.frame_id = parent
                tf.child_frame_id = child
                for axis, value in zip("xyz", translation):
                    setattr(tf.transform.translation, axis, float(value))
                for axis, value in zip("xyzw", rotation):
                    setattr(tf.transform.rotation, axis, float(value))
                transforms.append(tf)
            self.tf.publish(TFMessage(transforms=transforms))
        except (KeyError, TypeError, ValueError) as exc:
            log.warning("discarded malformed robot odom: %s", exc)

    def _on_lidar(self, msg):
        import numpy as np
        from sensor_msgs.msg import PointCloud2, PointField

        try:
            points = np.asarray(msg["data"]["data"]["points"], dtype="<f4").reshape(
                -1, 3
            )
            cloud = PointCloud2()
            cloud.header.stamp = self.node.get_clock().now().to_msg()
            cloud.header.frame_id = self.config["body"]["world_frame"]
            cloud.height = 1
            cloud.width = len(points)
            cloud.fields = [
                PointField(
                    name=axis, offset=i * 4, datatype=PointField.FLOAT32, count=1
                )
                for i, axis in enumerate("xyz")
            ]
            cloud.is_bigendian = False
            cloud.point_step = 12
            cloud.row_step = 12 * len(points)
            cloud.is_dense = bool(np.isfinite(points).all())
            cloud.data = points.tobytes()
            self.lidar.publish(cloud)
        except (KeyError, TypeError, ValueError) as exc:
            log.warning("discarded malformed robot lidar: %s", exc)

    def _on_lowstate(self, msg):
        from std_msgs.msg import String

        self.lowstate.publish(
            String(
                data=json.dumps(msg, ensure_ascii=False, default=lambda v: v.tolist())
            )
        )

    def check(self):
        if self.connection is not None:
            self.connection.check()

    def stop(self):
        for timer in self.timers:
            self.node.destroy_timer(timer)
        if self.connection is not None:
            self.connection.stop()
