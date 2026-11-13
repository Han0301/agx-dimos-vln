import math

import numpy as np


def room():
    grid = np.full((31, 41), -1, dtype=np.int8)
    grid[4:27, 4:37] = 0
    return grid


def test_local_goal_advances_towards_known_free_frontier():
    from dimos.plugins.active_search.frontier import FrontierPlanner

    planner = FrontierPlanner(lookahead_m=0.70)
    grid = room()
    origin = (-2.0, -1.5)
    robot = (0.05, 0.05)
    goal = planner.select(grid, origin_xy=origin, resolution=0.1, robot_xy=robot)
    assert goal is not None
    assert 0.3 <= math.dist(robot, goal.xy) <= 0.7 + 1e-8
    assert goal.path_xy[0] == robot
    assert planner.path_is_clear(
        grid, origin_xy=origin, resolution=0.1, robot_xy=robot, path_xy=[goal.xy]
    )


def test_no_goal_in_frontier_disconnected_by_wall():
    from dimos.plugins.active_search.frontier import FrontierPlanner

    grid = np.full((23, 42), 100, dtype=np.int8)
    grid[2:-2, 2:18] = 0
    grid[2:-2, 20:36] = 0
    grid[2:-2, 36:] = -1
    planner = FrontierPlanner()
    assert planner.select(
        grid, origin_xy=(0, 0), resolution=0.1, robot_xy=(0.85, 1.15)
    ) is None


def test_blacklist_failed_frontier_but_not_intermediate_waypoint():
    from dimos.plugins.active_search.frontier import FrontierPlanner

    planner = FrontierPlanner(lookahead_m=0.35)
    grid = room()
    pose = dict(origin_xy=(-2.0, -1.5), resolution=0.1, robot_xy=(0.05, 0.05))
    initial = planner.select(grid, **pose)
    assert initial is not None and not initial.final
    planner.mark_explored(initial)
    assert planner.select(grid, **pose).frontier_xy == initial.frontier_xy
    planner.mark_failed(initial)
    alternate = planner.select(grid, **pose)
    assert alternate is not None
    assert math.dist(alternate.frontier_xy, initial.frontier_xy) > 0.65
    planner.reset()
    assert planner.select(grid, **pose).frontier_xy == initial.frontier_xy


def test_path_validation_checks_intermediate_footprint_and_unknown():
    from dimos.plugins.active_search.frontier import FrontierPlanner

    grid = np.zeros((31, 41), dtype=np.int8)
    grid[0:16, 15] = 100
    grid[20:, 35:] = -1
    planner = FrontierPlanner()
    options = dict(origin_xy=(0, 0), resolution=0.1, robot_xy=(0.85, 1.15))
    assert not planner.path_is_clear(grid, **options, path_xy=[(2.35, 1.15)])
    assert not planner.path_is_clear(
        grid, **options, path_xy=[(0.85, 2.15), (3.25, 2.15)]
    )
    assert planner.path_is_clear(
        grid, **options, path_xy=[(0.85, 2.15), (2.35, 2.15), (2.35, 1.15)]
    )


def test_free_start_can_leave_when_nearby_unknown_reduces_clearance():
    from dimos.plugins.active_search.frontier import FrontierPlanner

    planner = FrontierPlanner()
    grid = np.full((21, 22), 0, dtype=np.int8)
    grid[:, 16:] = -1
    grid[10, 2] = -1
    pose = dict(origin_xy=(0, 0), resolution=0.1, robot_xy=(0.55, 1.05))
    goal = planner.select(grid, **pose)

    assert goal is not None
    assert math.dist(goal.xy, pose["robot_xy"]) >= planner.min_goal_m - 1e-8
    assert not planner.path_is_clear(grid, **pose, path_xy=[goal.xy])


def test_frontier_selection_crosses_known_free_corridor_without_clearance():
    from dimos.vln_task.search.frontier import FrontierPlanner

    grid = np.full((9, 16), 100, dtype=np.int8)
    grid[4, 2:12] = 0
    grid[4, 12:] = -1
    planner = FrontierPlanner()
    pose = dict(origin_xy=(0, 0), resolution=0.1, robot_xy=(0.25, 0.45))
    goal = planner.select(grid, **pose)

    assert goal is not None
    assert math.dist(goal.xy, (1.15, 0.45)) < 1e-8
    assert not planner.path_is_clear(grid, **pose, path_xy=[goal.xy])
