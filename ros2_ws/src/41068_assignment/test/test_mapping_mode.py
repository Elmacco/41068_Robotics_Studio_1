"""GoalWatchdog and Mapping mode's handling of goals that bring no new ground."""
import numpy as np
from geometry_msgs.msg import Pose2D

from coverage_explorer import CoverageExplorer, MapGrid
from mapping_mode import GoalWatchdog, MappingMode

RES = 0.1


def watchdog():
    return GoalWatchdog(stall_time=8.0, min_gain_area=0.5, min_progress=0.5, timeout=25.0)


def test_getting_closer_keeps_a_goal_alive_across_mapped_ground():
    dog = watchdog()
    dog.start(0.0, 100.0, distance=9.0)
    # 1 m/s over ground already mapped: no new ground, but closer every second
    for t in range(1, 9):
        assert dog.update(float(t), 100.0, 9.0 - t) is None


def test_new_ground_keeps_a_goal_alive_while_not_getting_closer():
    dog = watchdog()
    dog.start(0.0, 100.0, distance=1.0)
    # Turning on the spot at the goal, the camera sweeping new ground
    for t in range(1, 15):
        assert dog.update(float(t), 100.0 + t, 1.0) is None


def test_a_goal_with_neither_stalls_after_stall_time():
    # At the frontier behind the person: no closer, nothing new to see
    dog = watchdog()
    dog.start(0.0, 100.0, distance=3.0)
    dog.update(2.0, 100.2, 1.0)                 # flew 2 m closer, saw a sliver
    assert dog.update(9.9, 100.3, 1.0) is None
    assert 'no new ground' in dog.update(10.0, 100.3, 1.0)


def test_the_timeout_is_a_backstop_even_with_progress():
    dog = watchdog()
    dog.start(0.0, 100.0, distance=50.0)
    for t in range(1, 25):
        assert dog.update(float(t), 100.0 + t, 50.0 - t) is None
    assert 'timed out' in dog.update(25.0, 125.0, 25.0)


def test_a_goal_can_be_blacklisted_wider_than_the_default():
    grid = np.full((100, 100), -1, np.int8)
    grid[:, :50] = 0
    explorer = CoverageExplorer()
    explorer.update_map(MapGrid(grid, RES, 0.0, 0.0))
    explorer.find_frontiers()
    start = Pose2D(x=2.0, y=5.0)
    explorer.select_goal(start)
    x, y = explorer.goal_cell_xy_
    explorer.blacklist(x, y)                    # default 1.5 m: the frontier is 10 m long
    assert explorer.select_goal(start) is not None
    explorer.blacklist(x, y, radius=20.0)
    assert explorer.select_goal(start) is None


class FakeLogger:
    def __init__(self):
        self.lines = []

    def info(self, message):
        self.lines.append(message)

    def warn(self, message):
        self.lines.append(message)


class FakeManager:
    def __init__(self):
        self.logger = FakeLogger()

    def get_logger(self):
        return self.logger


def mapping_mode_at_goal(gain_m2, far=False):
    """A MappingMode that just reached a goal, having seen gain_m2 of new ground.

    far: the frontier is 24 m off, so the goal is a hop towards it.
    """
    if far:
        grid = np.full((300, 100), -1, np.int8)
        grid[:250, :] = 0
        start = Pose2D(x=5.0, y=1.0)
    else:
        grid = np.full((100, 100), -1, np.int8)
        grid[:, :50] = 0
        start = Pose2D(x=2.0, y=5.0)
    explorer = CoverageExplorer()
    explorer.update_map(MapGrid(grid, RES, 0.0, 0.0))
    explorer.find_frontiers()
    explorer.select_goal(start)
    assert explorer.is_waypoint_ == far

    mode = MappingMode.__new__(MappingMode)     # no ROS node behind it
    mode.manager = FakeManager()
    mode.explorer = explorer
    mode.min_goal_gain = 1.0
    mode.no_gain_blacklist_radius = 3.0
    mode.watchdog = watchdog()
    observed = explorer.exploration_stats()['observed_area_m2']
    mode.watchdog.start(0.0, observed - gain_m2, distance=0.0)
    mode.waiting_for_result = True
    mode.goal_handle = None
    return mode


def test_a_goal_reached_without_new_ground_is_blacklisted_wide():
    mode = mapping_mode_at_goal(gain_m2=0.3)
    mode.goal_finished(success=True)
    assert len(mode.explorer.blacklist_) == 1
    assert mode.explorer.blacklist_[0][2] == 3.0


def test_a_goal_reached_with_new_ground_is_not_blacklisted():
    mode = mapping_mode_at_goal(gain_m2=4.0)
    mode.goal_finished(success=True)
    assert mode.explorer.blacklist_ == []


def test_a_hop_reached_without_new_ground_is_not_blacklisted():
    mode = mapping_mode_at_goal(gain_m2=0.0, far=True)
    mode.goal_finished(success=True)
    assert mode.explorer.blacklist_ == []


def test_a_hop_far_from_any_frontier_is_not_stale():
    mode = mapping_mode_at_goal(gain_m2=0.0, far=True)
    mode.abandon_stale_goal()
    assert mode.waiting_for_result
