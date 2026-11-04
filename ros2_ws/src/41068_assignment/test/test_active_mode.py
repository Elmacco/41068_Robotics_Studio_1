import numpy as np

from active_mode import ActiveMode
from mapping_mode import MappingMode
from mission_manager import MissionManager
from mission_modes import HoverMode
from thermal_check import hot_pixels, thermal_confirms_person


class FakeParameter:
    def __init__(self, value):
        self.value = value


class FakeLogger:
    def warning(self, message):
        pass

    def info(self, message):
        pass


PARAMS = {'active_altitude': 3.0, 'active_dwell': 5.0, 'person_recent': 2.0}


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
        pass

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


def test_active_mode_finishes_after_dwell_at_altitude():
    manager = FakeManager(altitude=0.8)
    mode = ActiveMode(manager, person(), resume_mapping=True)
    mode.start()
    mode.step()
    manager.time += 4.0
    mode.step()
    assert not mode.finished
    manager.time += 1.5
    mode.step()
    assert mode.finished
