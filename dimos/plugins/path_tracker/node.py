"""Direct subscriptions to minimum's existing messages; no sensor/result relays."""

import json
import logging
import time

from .messages import (
    OccupancyGrid,
    decode_depth,
    decode_lidar,
    decode_result,
    odom_pose,
)
from .output import VelocityOutput
from .tracker import Tracker

log = logging.getLogger("dimos.path_tracker")
MOTION_DEPTH_MAX_AGE_S = 0.5


def source_stamp_ns(message):
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    return stamp.sec * 1_000_000_000 + stamp.nanosec if stamp is not None else None


class TrackerNode:
    def __init__(self, node, config, *, subscribe_results=True, inputs_ready=None):
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import OccupancyGrid as ROSGrid
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from sensor_msgs.msg import Image, PointCloud2
        from std_msgs.msg import String

        self.node = node
        self.cfg = config
        self.last_error = {}
        self.odom_z = None
        topics = config["topics"]
        self.output = VelocityOutput(node, topics["cmd_vel"], config["output_only"])
        self.tracker = Tracker(
            config,
            self.output,
            depth_timeout_s=None if subscribe_results else MOTION_DEPTH_MAX_AGE_S,
            inputs_ready=inputs_ready,
        )
        latest = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscriptions = [
            node.create_subscription(
                PoseStamped, topics["odom"], self._guard("odom", self.on_odom), latest
            ),
            node.create_subscription(
                Image, topics["depth"], self._guard("depth", self.on_depth), latest
            ),
        ]
        if subscribe_results:
            self.subscriptions.append(
                node.create_subscription(
                    String, topics["result"], self._guard("result", self.on_result), 10
                )
            )
        if topics["lidar"]:
            self.subscriptions.append(
                node.create_subscription(
                    PointCloud2,
                    topics["lidar"],
                    self._guard("lidar", self.on_lidar),
                    latest,
                )
            )
        if topics["costmap"]:
            self.subscriptions.append(
                node.create_subscription(
                    ROSGrid,
                    topics["costmap"],
                    self._guard("costmap", self.on_costmap),
                    10,
                )
            )

    def _guard(self, name, callback):
        def receive(message):
            try:
                callback(message)
            except (ValueError, KeyError, TypeError, IndexError, OverflowError) as exc:
                if name == "depth" and self.tracker.depth_timeout_s is not None:
                    self.tracker.update_depth(None)
                now = time.monotonic()
                if now - self.last_error.get(name, float("-inf")) >= 5:
                    if name == "result":
                        try:
                            payload = json.loads(message.data)
                        except (ValueError, TypeError):
                            payload = None
                        stamp = (
                            payload.get("request_stamp_ns")
                            if isinstance(payload, dict)
                            else None
                        )
                        log.warning(
                            "path rejected request_stamp_ns=%s reason=invalid_result error_type=%s",
                            stamp if type(stamp) is int else None,
                            type(exc).__name__,
                        )
                    else:
                        log.warning("Rejected %s input: %s", name, exc)
                    self.last_error[name] = now

        return receive

    def on_result(self, msg):
        self.tracker.submit(decode_result(msg.data))

    def on_odom(self, msg):
        odom, frame = odom_pose(msg)
        self.odom_z = msg.pose.position.z
        self.tracker.update_odom(odom, frame, stamp_ns=source_stamp_ns(msg))

    def on_depth(self, msg):
        self.tracker.update_depth(
            decode_depth(msg, self.cfg["depth_scale"]), stamp_ns=source_stamp_ns(msg),
        )

    def on_lidar(self, msg):
        with self.tracker.cv:
            odom, frame = self.tracker.odom, self.tracker.frame
        floor_z = None
        if self.tracker.depth_timeout_s is not None and self.odom_z is not None:
            floor_z = self.odom_z - 0.30
        self.tracker.update_scan(
            decode_lidar(msg, odom, frame, floor_z=floor_z), stamp_ns=source_stamp_ns(msg),
        )

    def on_costmap(self, msg):
        with self.tracker.cv:
            if not self.tracker.frame or msg.header.frame_id != self.tracker.frame:
                raise ValueError("costmap frame differs from odom")
        self.tracker.update_costmap(OccupancyGrid(msg))
