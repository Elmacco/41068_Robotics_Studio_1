import math

import numpy as np
from geometry_msgs.msg import Pose2D

from coverage_explorer import OCCUPIED_MIN, CoverageExplorer, MapGrid
from person_approach import (ApproachConfig, ApproachPlanner, Phase, _line_of_sight,
                             clearance_map, standoff_goal)

RES = 0.1


def make_grid(person=(5.0, 0.0), walls=(), size=30.0):
    """All free ground, origin at (-5, -size/2), person and walls as obstacles."""
    cells = int(size / RES)
    grid = MapGrid(np.zeros((cells, cells), dtype=np.int8), RES, -5.0, -size / 2)
    for (x0, y0, x1, y1) in walls + ((person[0] - 0.2, person[1] - 0.2,
                                      person[0] + 0.2, person[1] + 0.2),):
        r0, c0 = grid.world_to_grid(x0, y0)
        r1, c1 = grid.world_to_grid(x1, y1)
        grid.data[r0:r1 + 1, c0:c1 + 1] = 100
    return grid


def field_from(grid, x=0.0, y=0.0):
    explorer = CoverageExplorer()
    explorer.update_map(grid)
    return explorer.compute_distance_field(*grid.world_to_grid(x, y))


def goal_for(grid, person=(5.0, 0.0), avoid=()):
    return standoff_goal(person, grid, field_from(grid), clearance_map(grid),
                         ApproachConfig(), avoid)


def person(x=5.0, y=0.0, last_seen=0.0, pid='P1'):
    return {'id': pid, 'x': x, 'y': y, 'type': 'red', 'last_seen': last_seen}


def test_standoff_is_on_the_near_ring_facing_the_person():
    goal = goal_for(make_grid())
    assert math.isclose(math.hypot(goal.x - 5.0, goal.y), 1.5, abs_tol=1e-6)
    assert goal.x < 5.0                                 # drone side
    assert abs(goal.theta - math.atan2(-goal.y, 5.0 - goal.x)) < 1e-6


def test_line_of_sight_is_blocked_by_a_trunk_but_not_by_the_person():
    grid = make_grid(walls=((2.8, -0.3, 3.2, 0.3),))     # trunk at x = 3
    blocked = grid.data >= OCCUPIED_MIN
    assert not _line_of_sight(grid, blocked, (1.0, 0.0), (5.0, 0.0), 1.0)
    assert _line_of_sight(grid, blocked, (1.0, 2.0), (5.0, 2.0), 1.0)
    # The person's own cells (the last metre) do not block the view of them.
    assert _line_of_sight(grid, blocked, (3.5, 0.0), (5.0, 0.0), 1.0)


def test_unreachable_person_has_no_standoff():
    # A closed box of wall 4 m round the person.
    grid = make_grid(walls=((1.0, -4.0, 9.0, -3.8), (1.0, 3.8, 9.0, 4.0),
                            (1.0, -4.0, 1.2, 4.0), (8.8, -4.0, 9.0, 4.0)))
    assert goal_for(grid) is None


def test_retry_avoids_the_failed_standoff():
    grid = make_grid()
    first = goal_for(grid)
    second = goal_for(grid, avoid=[(first.x, first.y)])
    assert math.hypot(second.x - first.x, second.y - first.y) > 1.0


def test_far_person_is_reached_in_hops():
    grid = make_grid(person=(20.0, 0.0))
    planner = ApproachPlanner(ApproachConfig(), person(20.0), 0.0)
    goal = planner.plan(grid, field_from(grid), Pose2D(), 0.0)
    assert planner.is_hop and math.hypot(goal.x, goal.y) <= 9.0 + 0.2
    planner.on_goal_result(True, 10.0)
    assert planner.phase == Phase.PLANNING             # plan again from the hop


def test_visited_when_seen_again_from_the_standoff():
    grid = make_grid()
    planner = ApproachPlanner(ApproachConfig(), person(), 0.0)
    assert planner.plan(grid, field_from(grid), Pose2D(), 0.0) is not None
    planner.on_goal_result(True, 8.0)
    assert planner.phase == Phase.VERIFYING
    planner.tick([person(last_seen=8.5)], 9.0)          # too soon
    assert planner.phase == Phase.VERIFYING
    planner.tick([person(last_seen=9.4)], 9.6)
    assert planner.phase == Phase.DONE and planner.visited


def test_not_seen_retries_once_then_abandons():
    grid = make_grid()
    planner = ApproachPlanner(ApproachConfig(), person(), 0.0)
    first = planner.plan(grid, field_from(grid), Pose2D(), 0.0)
    planner.on_goal_result(True, 8.0)
    planner.tick([person(last_seen=1.0)], 14.0)         # stale sighting only
    assert planner.phase == Phase.PLANNING and planner.attempts == 2
    second = planner.plan(grid, field_from(grid), Pose2D(), 14.0)
    assert math.hypot(second.x - first.x, second.y - first.y) > 1.0
    planner.on_goal_result(False, 20.0)
    assert planner.phase == Phase.DONE and not planner.visited
    assert planner.reason == 'navigation failed'


def test_abandoned_when_no_standoff_appears():
    grid = make_grid(walls=((1.0, -4.0, 9.0, -3.8), (1.0, 3.8, 9.0, 4.0),
                            (1.0, -4.0, 1.2, 4.0), (8.8, -4.0, 9.0, 4.0)))
    planner = ApproachPlanner(ApproachConfig(), person(), 0.0)
    assert planner.plan(grid, field_from(grid), Pose2D(), 0.0) is None
    planner.tick([], 4.0)
    assert planner.phase == Phase.PLANNING
    planner.tick([], 6.0)
    assert planner.phase == Phase.DONE and not planner.visited


def test_replans_when_the_estimate_moves():
    grid = make_grid()
    planner = ApproachPlanner(ApproachConfig(), person(), 0.0)
    planner.plan(grid, field_from(grid), Pose2D(), 0.0)
    planner.update_person([person(x=6.0)])
    planner.tick([], 2.0)
    assert planner.phase == Phase.PLANNING and planner.attempts == 1


def test_approach_times_out_across_replans():
    grid = make_grid()
    planner = ApproachPlanner(ApproachConfig(), person(), 0.0)
    planner.plan(grid, field_from(grid), Pose2D(), 0.0)
    planner.update_person([person(x=6.0)])
    planner.tick([], 30.0)
    planner.plan(grid, field_from(grid), Pose2D(), 30.0)
    planner.tick([], 41.0)
    assert planner.attempts == 2
