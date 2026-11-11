"""Small bounded, world-aligned map from paired D455 depth observations."""

import math
import time
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

UNKNOWN = -1
FREE = 0
OCCUPIED = 100


def decode_world_lidar(msg, frame):
    """Return finite XYZ samples from the Go2's world-frame FLOAT32 cloud."""
    if not frame or msg.header.frame_id != frame:
        raise ValueError("LiDAR frame differs from exploration world frame")
    fields = {field.name: field for field in msg.fields}
    names = ("x", "y", "z")
    if any(
        axis not in fields
        or fields[axis].datatype != 7
        or fields[axis].count != 1
        or not 0 <= fields[axis].offset <= msg.point_step - 4
        for axis in names
    ):
        raise ValueError("Expected FLOAT32 XYZ fields in LiDAR cloud")
    if (
        msg.width < 0
        or msg.height <= 0
        or msg.row_step < msg.width * msg.point_step
        or len(msg.data) < msg.height * msg.row_step
    ):
        raise ValueError("Invalid LiDAR data size or strides")
    if msg.width == 0:
        return np.empty((0, 3), dtype=np.float32)
    dtype = np.dtype(
        {
            "names": names,
            "formats": [">f4" if msg.is_bigendian else "<f4"] * len(names),
            "offsets": [fields[axis].offset for axis in names],
            "itemsize": msg.point_step,
        }
    )
    points = np.ndarray(
        (msg.height, msg.width),
        dtype=dtype,
        buffer=memoryview(msg.data),
        strides=(msg.row_step, msg.point_step),
    )
    xyz = np.column_stack([points[axis].ravel() for axis in names])
    return xyz[np.isfinite(xyz).all(axis=1)]


@dataclass(frozen=True)
class GridSnapshot:
    """Cell values are indexed [y, x]. Origin is the lower-left cell edge."""

    grid: np.ndarray
    origin_xy: tuple[float, float]
    resolution: float
    frame: str
    stamp_ns: int


class DepthOccupancyMap:
    """Accumulate ground rays and blocking endpoints for a single search.

    This is a local projection for a roughly level floor. The configured
    base-relative floor height should match the robot's standing pose.
    """

    def __init__(
        self,
        camera,
        *,
        resolution=0.10,
        extent_m=10.0,
        footprint_radius=0.30,
        max_depth_m=4.0,
        max_pair_gap_ms=150.0,
        max_age_s=2.0,
        floor_z_base_m=-0.30,
    ):
        self.translation = np.asarray(camera["translation"], dtype=float)
        pitch = float(camera["pitch"])
        if (
            self.translation.shape != (3,)
            or not np.isfinite(self.translation).all()
            or not math.isfinite(pitch)
        ):
            raise ValueError("Invalid D455 installation transform")
        distances = (
            resolution, extent_m, footprint_radius, max_depth_m,
            max_pair_gap_ms, max_age_s,
        )
        if not all(math.isfinite(value) and value > 0 for value in distances):
            raise ValueError("Invalid exploration map distances")
        if not math.isfinite(floor_z_base_m):
            raise ValueError("Invalid exploration floor height")
        self.pitch_rotation = Rotation.from_rotvec([0, pitch, 0])
        self.resolution = resolution
        self.size = math.ceil(extent_m / resolution)
        self.footprint_radius = footprint_radius
        self.max_depth = max_depth_m
        self.max_pair_gap = max_pair_gap_ms
        self.max_age = max_age_s
        self.floor_z_base = floor_z_base_m
        self.grid = np.full((self.size, self.size), UNKNOWN, dtype=np.int8)
        self.origin = None
        self.frame = None
        self.last_stamp = 0
        self.updated_at = float("-inf")
        self.cloud_stamp = 0
        self.cloud_updated_at = float("-inf")
        self._geometry = None
        self._samples = None

    def _sample_geometry(self, observation):
        height, width = observation.depth.shape
        distortion = tuple(observation.distortion)
        geometry = (height, width, tuple(observation.k), distortion, observation.distortion_model)
        if geometry == self._geometry:
            return self._samples
        k = np.asarray(observation.k, dtype=float).reshape(3, 3)
        if not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0:
            raise ValueError("Invalid D455 intrinsics for exploration")
        cols, rows = np.meshgrid(
            np.arange(0, width, max(1, math.ceil(width / 112))),
            np.arange(0, height, max(1, math.ceil(height / 48))),
        )
        cols, rows = cols.ravel(), rows.ravel()
        if any(distortion):
            if observation.distortion_model != "plumb_bob" or len(distortion) != 5:
                raise ValueError("Unsupported nonzero D455 exploration distortion")
            import cv2

            xy = cv2.undistortPoints(
                np.column_stack((cols, rows)).astype(float),
                k,
                np.asarray(distortion, dtype=float),
            ).reshape(-1, 2)
        else:
            rays = np.linalg.solve(k, np.stack((cols, rows, np.ones_like(cols))))
            xy = (rays[:2] / rays[2]).T
        optical_to_base = np.column_stack((np.ones_like(cols), -xy[:, 0], -xy[:, 1]))
        self._samples = (rows, cols, self.pitch_rotation.apply(optical_to_base))
        self._geometry = geometry
        return self._samples

    def _cells(self, points):
        return np.floor((points - self.origin) / self.resolution).astype(np.intp)

    def _in_bounds(self, cells):
        return np.all((cells >= 0) & (cells < self.size), axis=-1)

    def _mark_occupied(self, points):
        cells = self._cells(points)
        x, y = cells[self._in_bounds(cells)].T
        self.grid[y, x] = OCCUPIED

    def _mark_floor_rays(self, start, floor_points):
        if not len(floor_points):
            return
        # Sample at under half a cell so diagonal traversals remain connected.
        step = self.resolution * 0.45
        delta = floor_points - start
        distances = np.linalg.norm(delta, axis=1)
        steps = max(1, int(math.ceil(np.max(distances) / step)))
        fraction = np.minimum(
            np.arange(steps + 1)[None, :] * step / np.maximum(distances[:, None], step),
            1.0,
        )
        cells = self._cells(start + delta[:, None, :] * fraction[:, :, None])
        valid = self._in_bounds(cells)
        xs, ys = np.moveaxis(np.clip(cells, 0, self.size - 1), -1, 0)
        clear_prefix = np.logical_and.accumulate(
            valid & (self.grid[ys, xs] != OCCUPIED), axis=1
        )
        self.grid[ys[clear_prefix], xs[clear_prefix]] = FREE

    def _mark_footprint(self, position):
        centre = self._cells(position)
        radius = math.ceil(self.footprint_radius / self.resolution)
        x0, y0 = np.maximum(0, centre - radius)
        x1, y1 = np.minimum(self.size, centre + radius + 1)
        if x0 >= x1 or y0 >= y1:
            return
        x = self.origin[0] + (np.arange(x0, x1) + 0.5) * self.resolution
        y = self.origin[1] + (np.arange(y0, y1) + 0.5) * self.resolution
        footprint = (x[None, :] - position[0]) ** 2 + (y[:, None] - position[1]) ** 2
        cells = self.grid[y0:y1, x0:x1]
        cells[(footprint <= self.footprint_radius**2) & (cells != OCCUPIED)] = FREE

    def update(self, observation, *, now=None):
        now = time.monotonic() if now is None else now
        stamp = observation.stamp_ns
        frame = observation.odom.header.frame_id
        if (
            not frame
            or (self.frame is not None and frame != self.frame)
            or stamp <= self.last_stamp
            or any(
                not math.isfinite(gap) or gap > self.max_pair_gap
                for gap in (observation.depth_gap_ms, observation.odom_gap_ms)
            )
        ):
            return False
        rows, cols, rays = self._sample_geometry(observation)
        depth = observation.depth[rows, cols]
        valid = np.isfinite(depth) & (depth >= 0.10) & (depth <= self.max_depth)
        if not np.any(valid):
            return False

        pose = observation.odom.pose
        position = np.array([pose.position.x, pose.position.y, pose.position.z])
        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        if not np.isfinite(position).all() or not np.isfinite(q).all():
            return False
        try:
            rotation = Rotation.from_quat(q)
        except ValueError:
            return False
        source = rotation.apply(self.translation) + position
        world_points = source + rotation.apply(rays[valid] * depth[valid, None])
        height = world_points[:, 2] - (position[2] + self.floor_z_base)
        floor = np.abs(height) <= 0.08
        obstacles = (height > 0.08) & (height <= 1.0)
        if not np.any(floor | obstacles):
            return False

        if self.origin is None:
            self.origin = position[:2] - (self.size // 2 + 0.5) * self.resolution
            self.frame = frame
        # Old occupied cells stay blocked for this search. A new line of
        # sight can fill unknown cells only before the first occupied cell.
        self._mark_occupied(world_points[obstacles, :2])
        self._mark_floor_rays(source[:2], world_points[floor, :2])
        self._mark_footprint(position[:2])
        self.last_stamp = stamp
        self.updated_at = now
        return True

    def update_cloud(self, points_xyz, *, pose_z, frame, stamp_ns, now=None):
        """Use registered Go2 voxel endpoints, without tracing sensor rays."""
        if (
            self.origin is None
            or frame != self.frame
            or stamp_ns <= self.cloud_stamp
            or not math.isfinite(pose_z)
        ):
            return False
        points = np.asarray(points_xyz, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("Expected Nx3 world LiDAR points")
        bounds = self.origin + self.size * self.resolution
        points = points[
            np.isfinite(points).all(axis=1)
            & np.all((points[:, :2] >= self.origin) & (points[:, :2] < bounds), axis=1)
        ]
        height = points[:, 2] - (pose_z + self.floor_z_base)
        floor = np.abs(height) <= 0.08
        obstacles = (height > 0.08) & (height <= 1.0)
        if not np.any(floor | obstacles):
            return False
        self._mark_occupied(points[obstacles, :2])
        floor_cells = self._cells(points[floor, :2])
        x, y = floor_cells[self._in_bounds(floor_cells)].T
        available = self.grid[y, x] != OCCUPIED
        self.grid[y[available], x[available]] = FREE
        self.cloud_stamp = stamp_ns
        self.cloud_updated_at = time.monotonic() if now is None else now
        return True

    def snapshot(self, *, now=None, frame=None, require_cloud=False):
        now = time.monotonic() if now is None else now
        if (
            self.origin is None
            or now - self.updated_at > self.max_age
            or (require_cloud and now - self.cloud_updated_at > self.max_age)
            or (frame is not None and frame != self.frame)
        ):
            return None
        grid = self.grid.copy()
        grid.setflags(write=False)
        return GridSnapshot(
            grid, tuple(self.origin), self.resolution, self.frame, self.last_stamp
        )
