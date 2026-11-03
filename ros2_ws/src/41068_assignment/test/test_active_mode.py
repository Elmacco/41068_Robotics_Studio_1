import numpy as np
from geometry_msgs.msg import Pose2D, PoseStamped

from active_mode import ActiveMode, thermal_confirms_person
from mapping_mode import MappingMode
from mission_manager import MissionManager


class FakeParameter:
    def __init__(self, value):
        self.value = value


class FakeLogger:
    def warning(self, message):
        pass

    def info(self, message):
        pass


class FakeManager:
    def __init__(self, thermal, thermal_current=True):
        self.latest_thermal = thermal
        self.latest_target_colour = 'red'
        self.latest_person_bbox = (2.5, 2.0, 5.0, 4.0)
        self.thermal_current = thermal_current
        self.switched_mode = None

    def get_parameter(self, name):
        return FakeParameter({'active_altitude': 3.0}.get(name))

    def thermal_is_current(self):
        return self.thermal_current

    def get_logger(self):
        return FakeLogger()

    def switch_to(self, mode):
        self.switched_mode = mode

    def person_detection_callback(self, pose):
        MissionManager.person_detection_callback(self, pose)


def test_thermal_confirmation_requires_twenty_hot_pixels():
    assert thermal_confirms_person(np.full((4, 5), 303.15))
    assert not thermal_confirms_person(np.full((4, 4), 303.15))


def test_mapping_detection_switches_to_active_when_both_sensors_confirm():
    manager = FakeManager(np.full((4, 5), 303.15))
    manager.mode = MappingMode.__new__(MappingMode)

    MissionManager.person_detection_callback(manager, Pose2D(x=2.0, y=3.0))

    assert isinstance(manager.switched_mode, ActiveMode)
    assert manager.switched_mode.last_person_confirmed
    assert manager.switched_mode.person_pose.x == 2.0
    assert manager.switched_mode.person_pose.y == 3.0


def test_stale_thermal_frame_does_not_interrupt_mapping():
    manager = FakeManager(np.full((4, 5), 303.15), thermal_current=False)
    manager.mode = MappingMode.__new__(MappingMode)

    MissionManager.person_detection_callback(manager, Pose2D(x=2.0, y=3.0))

    assert manager.switched_mode is None


def test_non_red_person_does_not_interrupt_mapping():
    manager = FakeManager(np.full((4, 5), 303.15))
    manager.latest_target_colour = 'none'
    manager.mode = MappingMode.__new__(MappingMode)

    MissionManager.person_detection_callback(manager, Pose2D(x=2.0, y=3.0))

    assert manager.switched_mode is None


def test_thermal_confirmation_is_limited_to_person_box():
    thermal = np.zeros((6, 6), dtype=np.float32)
    thermal[0:2, 0:2] = 303.15
    assert not thermal_confirms_person(thermal, box=(3, 3, 2, 2))
    assert thermal_confirms_person(thermal, box=(1, 1, 2, 2), min_hot_pixels=4)


def test_projector_pose_interrupts_mapping():
    manager = FakeManager(np.full((4, 5), 303.15))
    manager.mode = MappingMode.__new__(MappingMode)
    msg = PoseStamped()
    msg.pose.position.x = 2.0
    msg.pose.position.y = 3.0

    MissionManager.target_pose_callback(manager, msg)

    assert isinstance(manager.switched_mode, ActiveMode)
    assert manager.switched_mode.person_pose.x == 2.0
    assert manager.switched_mode.person_pose.y == 3.0
