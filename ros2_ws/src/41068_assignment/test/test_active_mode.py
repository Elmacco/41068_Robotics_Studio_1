import numpy as np
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import OccupancyGrid

from active_mode import ActiveMode
from coverage_explorer import CoverageExplorer
from mapping_mode import MappingMode
from mission_manager import MissionManager
from mission_modes import FollowConfig, HoverMode
from person_approach import ApproachConfig, Phase
from thermal_check import hot_pixels, thermal_confirms_person


class FakeParameter:
    def __init__(self, value):
        self.value = value


class FakeLogger:
    def warning(self, message):
        pass

    def info(self, message):
        pass


PARAMS = {'active_altitude': 3.0, 'person_recent': 2.0}


class FakeManager:
    """Just enough of MissionManager for check_people and ActiveMode."""

    def __init__(self, mode=None, people=(), altitude=0.8, now=100.0):
        self.mode = mode if mode is not None else MappingMode.__new__(MappingMode)
        self.people = list(people)
        self.handled_people = set()
        self.altitude = altitude
        self.time = now
        self.altitude_target = None
        self.switched_mode = None
        self.approach_config = ApproachConfig()
        self.follow_config = FollowConfig()
        self.explorer = CoverageExplorer()
        self.terrain_grid = None
        self.pose = Pose2D()
        self.cmds = []

    def get_parameter(self, name):
        return FakeParameter(PARAMS[name])

    def get_logger(self):
        return FakeLogger()

    def now(self):
        return self.time

    def switch_to(self, mode):
        self.switched_mode = mode
        self.mode = mode

    def publish_cmd(self, cmd):
        self.cmds.append(cmd)

    def get_pose(self):
        return self.pose

    def set_altitude(self, altitude):
        self.altitude_target = altitude

    def at_altitude(self, altitude):
        return abs(self.altitude - altitude) < 0.15

    def check_people(self):
        MissionManager.check_people(self)


def person(pid='P1', kind='red', last_seen=99.5):
    return {'id': pid, 'x': 2.0, 'y': 3.0, 'type': kind, 'last_seen': last_seen}


def test_thermal_confirmation_requires_twenty_hot_pixels():
    assert thermal_confirms_person(np.full((4, 5), 303.15))
    assert not thermal_confirms_person(np.full((4, 4), 303.15))


def test_thermal_confirmation_is_limited_to_person_box():
    thermal = np.zeros((6, 6), dtype=np.float32)
    thermal[0:2, 0:2] = 303.15
    assert hot_pixels(thermal, box=(3, 3, 2, 2)) == 0
    assert thermal_confirms_person(thermal, box=(1, 1, 2, 2), min_hot_pixels=4)


def test_red_person_interrupts_mapping_once():
    manager = FakeManager(people=[person()])
    manager.check_people()

    assert isinstance(manager.switched_mode, ActiveMode)
    assert manager.switched_mode.person['id'] == 'P1'
    assert manager.switched_mode.resume_mapping

    # Back in Mapping with P1 still in view: no second interruption.
    manager.mode, manager.switched_mode = MappingMode.__new__(MappingMode), None
    manager.check_people()
    assert manager.switched_mode is None


def test_non_red_or_out_of_view_person_does_not_interrupt_mapping():
    manager = FakeManager(people=[person(kind='person'), person('P2', last_seen=90.0)])
    manager.check_people()
    assert manager.switched_mode is None


def test_person_is_ignored_outside_mapping_and_active():
    manager = FakeManager(mode=HoverMode(None, 'IDLE'), people=[person()])
    manager.check_people()
    assert manager.switched_mode is None and not manager.handled_people


def test_waiting_active_mode_takes_the_person():
    manager = FakeManager(people=[person()])
    manager.mode = ActiveMode(manager)
    manager.check_people()
    assert manager.switched_mode is None
    assert manager.mode.person['id'] == 'P1' and not manager.mode.resume_mapping


def test_active_mode_does_not_climb_from_mapping_altitude():
    manager = FakeManager(altitude=0.8)
    mode = ActiveMode(manager, person(), resume_mapping=True)
    mode.start()
    assert manager.altitude_target == 0.8


def test_active_mode_still_descends_from_cruise_altitude():
    manager = FakeManager(altitude=8.0)
    mode = ActiveMode(manager)
    mode.start()
    assert manager.altitude_target == 3.0


class FakeNav:
    """Stands in for active_mode.NavGoal: records goals, results set by the test."""

    def __init__(self):
        self.goals = []
        self.result = None
        self.running = False

    def send(self, goal):
        self.goals.append(goal)
        self.running = True

    def take_result(self):
        result, self.result = self.result, None
        if result is not None:
            self.running = False
        return result

    def cancel(self):
        self.running = False


def open_field(person_xy=(2.0, 3.0)):
    """20 x 20 m of free ground at 0.1 m with the person as a 0.4 m obstacle."""
    msg = OccupancyGrid()
    msg.info.resolution = 0.1
    msg.info.width = msg.info.height = 200
    msg.info.origin.position.x = msg.info.origin.position.y = -10.0
    data = np.zeros((200, 200), dtype=np.int8)
    col, row = int((person_xy[0] + 10) / 0.1), int((person_xy[1] + 10) / 0.1)
    data[row - 2:row + 3, col - 2:col + 3] = 100
    msg.data = data.flatten().tolist()
    return msg


def test_active_mode_approaches_verifies_and_finishes():
    manager = FakeManager(altitude=0.8, people=[person(last_seen=99.5)])
    manager.terrain_grid = open_field()
    mode = ActiveMode(manager, person(), resume_mapping=True)
    mode.nav = FakeNav()
    mode.start()

    mode.step()
    assert len(mode.nav.goals) == 1 and mode.planner.phase == Phase.APPROACHING
    goal = mode.nav.goals[0]
    assert abs(np.hypot(goal.x - 2.0, goal.y - 3.0) - 1.5) < 1e-6

    manager.time = 110.0
    # Nav2 ends with any yaw: here 1 rad left of the person.
    manager.pose = Pose2D(x=goal.x, y=goal.y, theta=goal.theta + 1.0)
    mode.nav.result = True
    mode.step()
    assert mode.planner.phase == Phase.VERIFYING
    assert manager.cmds[-1].angular.z < 0.0                    # turning right to face them

    manager.time = 112.0
    manager.people = [person(last_seen=111.8)]
    mode.step()
    assert mode.finished and mode.planner.visited
