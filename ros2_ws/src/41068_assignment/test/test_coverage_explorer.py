"""CoverageExplorer on synthetic terrain occupancy grids."""
import math

import numpy as np
from geometry_msgs.msg import Pose2D

from coverage_explorer import CoverageExplorer, MapGrid

RES = 0.1


def pose(x, y, theta=0.0):
    p = Pose2D()
    p.x, p.y, p.theta = x, y, theta
    return p


def explorer_on(grid):
    explorer = CoverageExplorer()
    explorer.update_map(MapGrid(grid, RES, 0.0, 0.0))
    explorer.find_frontiers()
    return explorer


def test_goal_points_into_the_unobserved_side():
    grid = np.full((100, 100), -1, np.int8)     # 10 x 10 m, unobserved
    grid[:, :50] = 0                            # west half observed
    explorer = explorer_on(grid)
    assert len(explorer.frontiers_) == 1
    goal = explorer.select_goal(pose(2.0, 5.0))
    assert goal is not None
    assert goal.x < 5.0                         # stands on observed ground
    assert abs(goal.theta) < math.radians(20)   # facing east, into the unknown


def test_fully_observed_world_has_no_goal():
    explorer = explorer_on(np.zeros((100, 100), np.int8))
    assert explorer.frontiers_ == []
    assert explorer.select_goal(pose(5.0, 5.0)) is None
    assert explorer.exploration_stats()['coverage'] == 1.0


def test_goals_beyond_max_goal_distance_are_skipped():
    grid = np.full((300, 100), -1, np.int8)     # 10 m wide, 30 m long
    grid[:, :50] = 0
    explorer = explorer_on(grid)
    goal = explorer.select_goal(pose(2.0, 1.0))
    assert goal is not None
    assert math.hypot(goal.x - 2.0, goal.y - 1.0) <= explorer.max_goal_distance + 1.0


def far_frontier_grid():
    """10 x 30 m, mapped up to y = 25 m, with a wall at y = 10 m open at its east end."""
    grid = np.full((300, 100), -1, np.int8)
    grid[:250, :] = 0
    grid[100:103, :70] = 100
    return grid


def test_a_frontier_out_of_range_is_reached_in_hops():
    explorer = explorer_on(far_frontier_grid())
    start = pose(2.0, 1.0)
    goal = explorer.select_goal(start)
    assert goal is not None
    assert explorer.is_waypoint_
    assert explorer.target_pose_.y > 23.0                    # the frontier end
    assert math.hypot(goal.x - start.x, goal.y - start.y) <= explorer.max_goal_distance
    assert goal.x > 6.0                                      # heading for the gap in the wall
    row, col = explorer.map_.world_to_grid(goal.x, goal.y)
    assert explorer.clearance_[row, col] >= explorer.min_clearance


def test_hops_lead_to_the_frontier():
    explorer = explorer_on(far_frontier_grid())
    here = pose(2.0, 1.0)
    for _ in range(5):
        goal = explorer.select_goal(here)
        assert goal is not None
        if not explorer.is_waypoint_:
            break
        here = goal
    assert not explorer.is_waypoint_
    assert goal.y > 23.0


def test_trunk_shadow_is_not_worth_a_trip():
    grid = np.zeros((100, 100), np.int8)
    grid[48:52, 48:52] = 100                    # trunk
    grid[52:56, 48:52] = -1                     # small sealed shadow behind it
    explorer = explorer_on(grid)
    assert explorer.frontiers_ == []


def test_blacklisted_frontier_is_not_offered_again():
    grid = np.full((100, 100), -1, np.int8)
    grid[:, :50] = 0
    explorer = explorer_on(grid)
    goal = explorer.select_goal(pose(2.0, 5.0))
    x, y = explorer.goal_cell_xy_
    explorer.blacklist_radius = 20.0            # blacklist covers the whole frontier
    explorer.blacklist(x, y)
    assert explorer.select_goal(pose(2.0, 5.0)) is None
    assert goal is not None
