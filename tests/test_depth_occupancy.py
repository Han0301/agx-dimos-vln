"""Depth-to-grid geometry for the independent exploration mapper."""

import math
import struct
from types import SimpleNamespace as NS

import numpy as np
import pytest


def observation(stamp=10, frame="world", xy=(0.0, 0.0), yaw=0.0, gap_ms=0.0):
    return NS(
        stamp_ns=stamp,
        depth=np.zeros((31, 41), dtype=np.float32),
        k=(25, 0, 20, 0, 25, 14, 0, 0, 1),
        distortion=(),
        distortion_model="plumb_bob",
        depth_gap_ms=gap_ms,
        odom_gap_ms=gap_ms,
        odom=NS(
            header=NS(frame_id=frame),
            pose=NS(
                position=NS(x=xy[0], y=xy[1], z=0.0),
                orientation=NS(x=0, y=0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)),
            ),
        ),
    )


def mapper(**settings):
    from dimos.plugins.active_search.occupancy import DepthOccupancyMap

    return DepthOccupancyMap(
        {"translation": [0.0, 0.0, 0.5], "pitch": 0.0},
        floor_z_base_m=0.0,
        **settings,
    )


def cell(snapshot, x, y):
    ox, oy = snapshot.origin_xy
    return snapshot.grid[math.floor((y - oy) / snapshot.resolution),
                         math.floor((x - ox) / snapshot.resolution)]


def test_depth_only_clears_observed_rays_and_preserves_unknown():
    grid = mapper()
    sample = observation()
    # Camera at z=.5, this lower pixel intersects the floor 1.25 m ahead.
    sample.depth[24, 20] = 1.25
    assert grid.update(sample, now=10.0)
    result = grid.snapshot(now=10.1, frame="world")
    assert result.stamp_ns == 10
    assert cell(result, 0.4, 0) == 0
    assert cell(result, 1.25, 0) == 0
    assert cell(result, 2.0, 0) == -1
    assert cell(result, 0, 1.5) == -1


def test_nearer_obstacle_wins_over_free_evidence_behind_it():
    grid = mapper()
    sample = observation()
    sample.depth[14, 20] = 0.66  # occupied hit above the floor
    sample.depth[24, 20] = 1.25  # same horizontal ray, farther floor hit
    assert grid.update(sample, now=10.0)
    result = grid.snapshot(now=10.1)
    assert cell(result, 0.66, 0) == 100
    assert cell(result, 0.95, 0) == -1


def test_world_transform_applies_position_and_heading():
    grid = mapper()
    sample = observation(xy=(2.0, 1.0), yaw=math.pi / 2)
    sample.depth[14, 20] = 1.0
    assert grid.update(sample, now=10.0)
    result = grid.snapshot(now=10.1)
    assert cell(result, 2.0, 2.0) == 100
    assert cell(result, 3.0, 1.0) == -1


def test_stale_duplicate_invalid_and_foreign_inputs_do_not_refresh_map():
    grid = mapper(max_age_s=0.5)
    first = observation()
    first.depth[24, 20] = 1.25
    assert grid.update(first, now=10.0)
    assert not grid.update(first, now=10.4)
    invalid = observation(stamp=11)
    assert not grid.update(invalid, now=10.4)
    foreign = observation(stamp=12, frame="odom")
    foreign.depth[24, 20] = 1.25
    assert not grid.update(foreign, now=10.4)
    assert grid.snapshot(now=10.4, frame="odom") is None
    assert grid.snapshot(now=10.51, frame="world") is None


def test_depth_pair_gap_does_not_prevent_exploration_map_update():
    grid = mapper()
    sample = observation(gap_ms=559.4)
    sample.depth[24, 20] = 1.25
    assert grid.update(sample, now=10.)
    assert grid.snapshot(now=10.).stamp_ns == sample.stamp_ns


def test_exploration_lidar_warns_on_pair_gap_but_keeps_recent_world_cloud(monkeypatch, caplog):
    from dimos.vln_task.search import exploration, occupancy

    monkeypatch.setattr(exploration.time, "monotonic", lambda: 10.)
    monkeypatch.setattr(occupancy, "decode_world_lidar", lambda *_: np.array([[1., 2., 3.]]))
    cloud = exploration.ExplorationCloud(None, None)
    cloud._receive(NS(header=NS(stamp=NS(sec=2, nanosec=0))))
    odom = NS(header=NS(stamp=NS(sec=1, nanosec=0), frame_id="world"), pose=NS(position=NS(z=0.)))

    sample = cloud.read(odom)

    assert sample.stamp_ns == 2_000_000_000
    assert "1000.0 ms" in caplog.text


def test_world_voxel_points_never_ray_carve_and_obstacles_dominate():
    grid = mapper(max_age_s=0.5)
    first = observation()
    first.depth[14, 20] = 1.0
    assert grid.update(first, now=10.0)
    world_cloud = [
        [0.8, 0.8, 0.0],  # actual floor evidence
        [1.2, 0.8, 0.0],
        [1.2, 0.8, 0.2],  # same cell obstacle evidence wins
        [0.8, -0.8, -1.0],  # no evidence in the operating height band
    ]
    assert grid.snapshot(now=10.1, require_cloud=True) is None
    assert grid.update_cloud(world_cloud, pose_z=0.0, frame="world", stamp_ns=21, now=10.2)
    assert not grid.update_cloud(world_cloud, pose_z=0.0, frame="world", stamp_ns=21, now=10.4)
    result = grid.snapshot(now=10.4, require_cloud=True)
    assert cell(result, 0.8, 0.8) == 0
    assert cell(result, 1.2, 0.8) == 100
    assert cell(result, 0.5, 0.5) == -1
    assert cell(result, 0.8, -0.8) == -1
    assert grid.snapshot(now=10.51) is None


def test_decode_world_cloud_honors_row_padding_field_order_and_endian():
    from dimos.plugins.active_search.occupancy import decode_world_lidar

    fields = [NS(name=axis, offset=index * 4, datatype=7, count=1)
              for index, axis in enumerate(("y", "z", "x"))]
    point1 = struct.pack(">fff", 2, 3, 1)
    point2 = struct.pack(">fff", 5, 6, 4)
    message = NS(header=NS(frame_id="world"), fields=fields, width=1, height=2,
                 point_step=12, row_step=16, data=point1 + b"pad!" + point2 + b"pad!",
                 is_bigendian=True)
    np.testing.assert_allclose(decode_world_lidar(message, "world"), [[1, 2, 3], [4, 5, 6]])
    with pytest.raises(ValueError, match="frame"):
        decode_world_lidar(message, "odom")
    message.data = point1
    with pytest.raises(ValueError, match="data"):
        decode_world_lidar(message, "world")
