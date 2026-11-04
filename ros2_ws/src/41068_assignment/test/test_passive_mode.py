"""Passive mode's trail patrol: Nav2 goals stepped along the trail's waypoints."""
import math

from geometry_msgs.msg import Pose2D

from passive_mode import PassiveMode
from test_active_mode import FakeManager, FakeNav, open_field

TRAIL = [(0.0, 0.0), (2.0, 0.0), (4.0, 0.0), (4.0, 2.0)]


def patrol(waypoints=TRAIL, pose=(0.0, 0.0)):
    manager = FakeManager(altitude=0.8)
    manager.pose = Pose2D(x=pose[0], y=pose[1], theta=0.0)
    mode = PassiveMode(manager, 'T1', waypoints, 0.8)
    mode.nav = FakeNav()
    manager.mode = mode
    mode.start()
    return manager, mode


def test_flies_each_waypoint_facing_along_the_trail():
    manager, mode = patrol()
    mode.step()
    # The first waypoint is under the drone, so it heads for the second.
    goal = mode.nav.goals[-1]
    assert (goal.x, goal.y, goal.theta) == (2.0, 0.0, 0.0)

    # Within advance_radius of it: the next goal goes out before arriving.
    manager.pose = Pose2D(x=1.2, y=0.0, theta=0.0)
    mode.step()
    goal = mode.nav.goals[-1]
    assert (goal.x, goal.y) == (4.0, 0.0)
    assert math.isclose(goal.theta, math.pi / 2)

    # The last waypoint faces on along the trail's last stretch.
    manager.pose = Pose2D(x=3.5, y=0.0, theta=0.0)
    mode.step()
    goal = mode.nav.goals[-1]
    assert (goal.x, goal.y) == (4.0, 2.0) and math.isclose(goal.theta, math.pi / 2)

    manager.pose = Pose2D(x=4.0, y=1.5, theta=0.0)
    assert mode.step() == 'reached the end of the trail' and mode.finished


def test_waits_for_altitude_first():
    manager, mode = patrol()
    manager.altitude = 0.3
    mode.step()
    assert manager.altitude_target == 0.8 and not mode.nav.goals


def test_skips_an_unreachable_waypoint_then_gives_up():
    waypoints = [(float(x), 0.0) for x in range(2, 20, 2)]
    manager, mode = patrol(waypoints)
    mode.step()
    assert mode.nav.goals[-1].x == 2.0

    mode.nav.result = False
    mode.step()
    assert mode.nav.goals[-1].x == 4.0 and not mode.finished

    for _ in range(mode.config.max_skips):
        mode.nav.result = False
        status = mode.step()
    assert mode.finished and 'gave up' in status


def test_far_trail_end_is_reached_in_hops():
    manager, mode = patrol([(9.0, 0.0), (9.0, 2.0)], pose=(-6.0, 0.0))
    manager.terrain_grid = open_field(person_xy=(-9.0, -9.0))
    mode.step()
    goal = mode.nav.goals[-1]
    assert mode.is_hop
    assert abs(math.hypot(goal.x + 6.0, goal.y) - manager.approach_config.max_goal_distance) < 0.2

    # Hop reached: the trail end is now in range, so it is sent directly.
    manager.pose = Pose2D(x=goal.x, y=goal.y, theta=0.0)
    mode.nav.result = True
    mode.step()
    assert not mode.is_hop and (mode.nav.goals[-1].x, mode.nav.goals[-1].y) == (9.0, 0.0)


def test_remaining_resumes_from_the_waypoint_it_was_heading_to():
    manager, mode = patrol()
    mode.step()
    manager.pose = Pose2D(x=1.2, y=0.0, theta=0.0)
    mode.step()
    rest = mode.remaining()
    assert rest.waypoints == TRAIL[2:] and rest.name == 'PASSIVE T1'
