"""Build a local ROS costmap from the Go2's world-frame voxel cloud."""

import logging
import math
import time

import numpy as np
from scipy import ndimage

from dimos.plugins.runtime_log import event, recording_enabled

from .search.occupancy import decode_world_lidar

log = logging.getLogger(__name__)

RESOLUTION = 0.05
EXTENT_M = 10.0


def lidar_height_costs(points, centre_xy):
    """Use dimos_simple's height-gradient costs inside a bounded local window."""
    size = math.ceil(EXTENT_M / RESOLUTION)
    origin = (np.floor(np.asarray(centre_xy) / RESOLUTION) - size // 2) * RESOLUTION
    inside = np.isfinite(points).all(axis=1) & np.all(
        (points[:, :2] >= origin) & (points[:, :2] < origin + size * RESOLUTION),
        axis=1,
    )
    points = points[inside]
    if not len(points):
        raise ValueError("No finite LiDAR points within the costmap")

    cells = np.floor((points[:, :2] - origin) / RESOLUTION).astype(np.intp)
    x, y = np.clip(cells, 0, size - 1).T
    low = np.full((size, size), np.inf)
    high = np.full_like(low, -np.inf)
    z = points[:, 2] - np.median(points[:, 2])
    np.minimum.at(low, (y, x), z)
    np.maximum.at(high, (y, x), z)
    observed = np.isfinite(low)
    heights = np.where(high - low > 0.6, low, high)
    heights[~observed] = 0.0

    weights = ndimage.gaussian_filter(observed.astype(float), sigma=1.0)
    smoothed = ndimage.gaussian_filter(heights, sigma=1.0)
    fill = ~observed & (weights > 0.01)
    heights[fill] = smoothed[fill] / weights[fill]
    valid = ndimage.binary_erosion(observed | fill)
    step = np.hypot(ndimage.sobel(heights, axis=0), ndimage.sobel(heights, axis=1)) / 8
    cost = np.where(step < 0.05, 0.0, np.minimum(step / 0.15, 1.0) * 100)
    return np.where(valid, cost, -1).astype(np.int8), origin


class LidarCostmap:
    def __init__(self, node, tracker, lidar_topic, map_topic):
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import PointCloud2

        self.tracker = tracker
        self.last_update_log = None
        self.message_type = OccupancyGrid
        self.publisher = node.create_publisher(OccupancyGrid, map_topic, 1)
        self.subscription = node.create_subscription(
            PointCloud2, lidar_topic, self._receive, qos_profile_sensor_data
        )

    def _receive(self, msg):
        recording = recording_enabled()
        started = time.monotonic() if recording else None
        with self.tracker.cv:
            odom, frame = self.tracker.odom, self.tracker.frame
            if recording:
                odom_stamp_ns = self.tracker.odom_stamp_ns
                odom_age_ms = ((self.tracker.clock() - self.tracker.odom_received) * 1000
                               if odom is not None else None)
        stamp = msg.header.stamp

        if recording:
            def report(outcome, reason=None, **fields):
                now = time.monotonic()
                state = (outcome, reason)
                if self.last_update_log is not None:
                    previous, last = self.last_update_log
                    if previous == state and now - last < 1.0:
                        return
                self.last_update_log = state, now
                event(
                    log, "lidar_costmap_update", outcome=outcome, reason=reason,
                    lidar_stamp_ns=stamp.sec * 1_000_000_000 + stamp.nanosec,
                    lidar_frame=msg.header.frame_id, odom_frame=frame, odom_stamp_ns=odom_stamp_ns,
                    odom_age_ms=odom_age_ms, elapsed_ms=(now - started) * 1000, **fields,
                )

        if odom is None:
            if recording:
                report("rejected", "missing_odom")
            return
        if msg.header.frame_id != frame:
            if recording:
                report("rejected", "frame_mismatch")
            return
        if not (stamp.sec or stamp.nanosec):
            if recording:
                report("rejected", "missing_stamp")
            return
        try:
            points = decode_world_lidar(msg, frame)
            cost, origin = lidar_height_costs(points, odom[:2])
        except (TypeError, ValueError) as exc:
            log.warning("LiDAR costmap skipped: %s", exc)
            if recording:
                report("rejected", "invalid_cloud_or_grid", error_type=type(exc).__name__, detail=str(exc))
            return
        grid = self.message_type()
        grid.header = msg.header
        grid.info.resolution = RESOLUTION
        grid.info.height, grid.info.width = cost.shape
        grid.info.origin.position.x, grid.info.origin.position.y = map(float, origin)
        grid.info.origin.orientation.w = 1.0
        grid.data = cost.ravel().tolist()
        self.publisher.publish(grid)
        if recording:
            report("published", point_count=len(points), origin_xy=tuple(map(float, origin)),
                   grid_shape=cost.shape, resolution_m=RESOLUTION)
