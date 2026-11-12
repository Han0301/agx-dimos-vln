"""Reachable occupancy-grid frontiers for native XNavDP point goals."""

import heapq
import logging
import math
from collections import Counter, deque
from dataclasses import dataclass

import numpy as np
from scipy.ndimage import distance_transform_edt

from dimos.plugins.runtime_log import event, recording_enabled

logger = logging.getLogger("dimos.plugins.active_search.frontier")

_NEIGHBORS = (
    (-1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0), (1, 0, 1.0),
    (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
    (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2)),
)
_HALF_DIAGONAL = math.sqrt(0.5)


@dataclass(frozen=True)
class FrontierGoal:
    xy: tuple[float, float]
    frontier_xy: tuple[float, float]
    path_xy: tuple[tuple[float, float], ...]
    final: bool


def _open(grid, row, col):
    return (
        0 <= row < grid.shape[0]
        and 0 <= col < grid.shape[1]
        and grid[row, col]
    )


def _line_clear(grid, start, end):
    """Visit each cell crossed by the segment, including both corner neighbors."""
    x, y = start
    end_x, end_y = end
    col, row = math.floor(x), math.floor(y)
    last_col, last_row = math.floor(end_x), math.floor(end_y)
    if not _open(grid, row, col) or not _open(grid, last_row, last_col):
        return False
    dx, dy = end_x - x, end_y - y
    step_x = 1 if dx > 0 else -1
    step_y = 1 if dy > 0 else -1
    stride_x = 1 / abs(dx) if dx else math.inf
    stride_y = 1 / abs(dy) if dy else math.inf
    next_x = (col + (dx > 0) - x) / dx if dx else math.inf
    next_y = (row + (dy > 0) - y) / dy if dy else math.inf
    while (row, col) != (last_row, last_col):
        crossing_x = next_x if col != last_col else math.inf
        crossing_y = next_y if row != last_row else math.inf
        if math.isclose(crossing_x, crossing_y, rel_tol=0, abs_tol=1e-12):
            if not _open(grid, row + step_y, col) or not _open(grid, row, col + step_x):
                return False
            col += step_x
            row += step_y
            next_x += stride_x
            next_y += stride_y
        elif crossing_x < crossing_y:
            col += step_x
            next_x += stride_x
        else:
            row += step_y
            next_y += stride_y
        if not _open(grid, row, col):
            return False
    return True


class FrontierPlanner:
    """Choose a short reachable waypoint approaching a known/unknown boundary."""

    def __init__(
        self, *, clearance_m=0.30, lookahead_m=1.0, min_goal_m=0.30,
        explored_radius_m=0.65,
    ):
        values = (clearance_m, lookahead_m, min_goal_m, explored_radius_m)
        if not all(math.isfinite(value) and value > 0 for value in values):
            raise ValueError("frontier distances must be positive and finite")
        if min_goal_m <= 0.20 or lookahead_m < min_goal_m:
            raise ValueError("frontier lookahead must exceed the 0.20 m tracker tolerance")
        self.clearance_m = clearance_m
        self.lookahead_m = lookahead_m
        self.min_goal_m = min_goal_m
        self.explored_radius_m = explored_radius_m
        self.excluded = deque(maxlen=128)

    def reset(self):
        self.excluded.clear()

    def mark_explored(self, goal):
        if goal.final:
            self.excluded.append(goal.frontier_xy)

    def mark_failed(self, goal):
        self.excluded.append(goal.frontier_xy)

    def _masks(self, grid, resolution):
        grid = np.asarray(grid)
        if grid.ndim != 2 or not grid.size or not math.isfinite(resolution) or resolution <= 0:
            raise ValueError("invalid frontier grid or resolution")
        free = (grid >= 0) & (grid < 50)
        # Include half the blocked cell diagonal, and treat outside the map as blocked.
        cells = self.clearance_m / resolution + _HALF_DIAGONAL
        clearance = distance_transform_edt(np.pad(free, 1))[1:-1, 1:-1]
        return free, clearance >= cells, cells

    def path_is_clear(self, grid, *, origin_xy, resolution, robot_xy, path_xy):
        _, safe, _ = self._masks(grid, resolution)
        points = np.asarray([robot_xy, *path_xy], dtype=float)
        if points.shape != (len(points), 2) or len(points) < 2 or not np.isfinite(points).all():
            return False
        points = (points - origin_xy) / resolution
        return all(_line_clear(safe, start, end) for start, end in zip(points, points[1:]))

    def select(self, grid, *, origin_xy, resolution, robot_xy, allow_frontier=None):
        grid = np.asarray(grid)
        rejections = Counter() if recording_enabled() else None

        if rejections is not None:
            def report(reason, **fields):
                event(
                    logger, "frontier_selection", reason=reason, grid_shape=grid.shape,
                    origin_xy=tuple(map(float, origin_xy)), resolution_m=resolution,
                    clearance_m=self.clearance_m, lookahead_m=self.lookahead_m,
                    min_goal_m=self.min_goal_m, rejections=dict(rejections), **fields,
                )

        try:
            free, safe, cells = self._masks(grid, resolution)
        except ValueError:
            if rejections is not None and math.isfinite(resolution):
                report("invalid_grid")
            raise
        x, y = (np.asarray(robot_xy) - origin_xy) / resolution
        if not math.isfinite(x) or not math.isfinite(y):
            if rejections is not None:
                report("invalid_robot_position")
            return None
        start = (math.floor(y), math.floor(x))
        if not _open(safe, *start):
            if rejections is not None:
                row, col = start
                inside = 0 <= row < grid.shape[0] and 0 <= col < grid.shape[1]
                cost = float(grid[start]) if inside else None
                if not inside:
                    reason = "start_outside_map"
                elif not math.isfinite(cost):
                    reason, cost = "start_invalid_cell", None
                elif cost < 0:
                    reason = "start_unknown"
                elif cost >= 50:
                    reason = "start_occupied"
                else:
                    reason = "start_clearance"
                report(reason, start_cell_xy=(col, row), start_cost=cost)
            return None

        unknown = np.pad(grid < 0, 1)
        frontiers = free & (
            unknown[:-2, 1:-1] | unknown[2:, 1:-1]
            | unknown[1:-1, :-2] | unknown[1:-1, 2:]
        )
        if not frontiers.any():
            if rejections is not None:
                report("no_frontier_cells", start_cell_xy=start[::-1])
            return None
        distance, nearest = distance_transform_edt(~frontiers, return_indices=True)
        candidates = safe & (distance <= cells + math.sqrt(2))

        def world(cell):
            row, col = cell
            return (origin_xy[0] + (col + 0.5) * resolution,
                    origin_xy[1] + (row + 0.5) * resolution)

        queue = [(0.0, start)]
        distances = {start: 0.0}
        parents = {}
        reachable_candidates = 0
        while queue:
            traveled, cell = heapq.heappop(queue)
            if traveled > distances[cell]:
                continue
            row, col = cell
            if candidates[cell]:
                frontier = tuple(nearest[:, row, col])
                frontier_xy = world(frontier)
                if rejections is not None:
                    reachable_candidates += 1
                if not all(math.dist(frontier_xy, old) > self.explored_radius_m for old in self.excluded):
                    if rejections is not None:
                        rejections["excluded_frontier"] += 1
                elif allow_frontier is not None and not allow_frontier(frontier_xy):
                    if rejections is not None:
                        rejections["filtered_frontier"] += 1
                elif not _line_clear(free, (col + 0.5, row + 0.5),
                                     (frontier[1] + 0.5, frontier[0] + 0.5)):
                    if rejections is not None:
                        rejections["frontier_line_blocked"] += 1
                else:
                    route = []
                    node = cell
                    while node != start:
                        route.append(node)
                        node = parents[node]
                    route.reverse()
                    goal = self._waypoint(route, safe, (x, y), tuple(robot_xy),
                                          frontier_xy, resolution, world, rejections)
                    if goal is not None:
                        if rejections is not None:
                            report("selected", start_cell_xy=start[::-1],
                                   reachable_candidate_cells=reachable_candidates,
                                   goal_xy=goal.xy, frontier_xy=goal.frontier_xy, final=goal.final)
                        return goal
                    if rejections is not None:
                        rejections["no_usable_waypoint"] += 1

            for dy, dx, step in _NEIGHBORS:
                next_cell = row + dy, col + dx
                if not _open(safe, *next_cell):
                    continue
                if dy and dx and not (_open(safe, row + dy, col) and _open(safe, row, col + dx)):
                    continue
                cost = traveled + step
                if cost < distances.get(next_cell, math.inf):
                    distances[next_cell] = cost
                    parents[next_cell] = cell
                    heapq.heappush(queue, (cost, next_cell))
        if rejections is not None:
            candidate_cells = int(np.count_nonzero(candidates))
            if not candidate_cells:
                reason = "no_safe_frontier_cells"
            elif not reachable_candidates:
                reason = "no_reachable_candidate_cells"
            else:
                reason = "all_reachable_candidates_rejected"
            report(reason, start_cell_xy=start[::-1], frontier_cells=int(np.count_nonzero(frontiers)),
                   candidate_cells=candidate_cells, reachable_candidate_cells=reachable_candidates,
                   visited_cells=len(distances))
        return None

    def _waypoint(self, route, safe, start, robot_xy, frontier_xy, resolution, world, rejections=None):
        path = (robot_xy, *(world(cell) for cell in route))
        best = None
        traveled = 0.0
        for index, (row, col) in enumerate(route, start=1):
            traveled += math.dist(path[index - 1], path[index])
            if traveled > self.lookahead_m + 1e-9:
                if rejections is not None:
                    rejections["waypoint_beyond_lookahead"] += 1
                break
            if math.dist(robot_xy, path[index]) + 1e-9 < self.min_goal_m:
                if rejections is not None:
                    rejections["waypoint_too_close"] += 1
            elif not _line_clear(safe, start, (col + 0.5, row + 0.5)):
                if rejections is not None:
                    rejections["waypoint_line_blocked"] += 1
            else:
                best = index
        if best is None:
            if rejections is not None and not route:
                rejections["empty_route"] += 1
            return None
        return FrontierGoal(path[best], frontier_xy, path, best == len(path) - 1)
