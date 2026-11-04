import math
import random

from person_registry import PersonRegistry, RegistryConfig


def feed(reg, x, y, colour='none', n=5, noise=0.2, observer=(0.0, 0.0), seed=0):
    rnd = random.Random(seed)
    for i in range(n):
        reg.update(x + rnd.gauss(0, noise), y + rnd.gauss(0, noise), colour, 0.8, observer, float(i))


def test_noisy_sightings_merge_into_one_person():
    reg = PersonRegistry()
    feed(reg, 5.0, 3.0, n=10)
    people = reg.confirmed()
    assert len(people) == 1
    assert abs(people[0].x - 5.0) < 0.3 and abs(people[0].y - 3.0) < 0.3


def test_two_people_stay_separate_with_stable_ids():
    reg = PersonRegistry()
    feed(reg, 5.0, 3.0)
    feed(reg, -10.0, 8.0, colour='red', seed=1)
    ids = [p.id for p in reg.confirmed()]
    assert ids == ['P1', 'P2']
    assert reg.people['P2'].colour == 'red'


def test_single_sighting_is_not_reported():
    reg = PersonRegistry()
    reg.update(1.0, 1.0, 'none', 0.9, (0, 0), 0.0)
    assert reg.confirmed() == [] and reg.get('P1') is None


def test_colour_needs_majority_red():
    reg = PersonRegistry()
    for c in ['red', 'none', 'none', 'none', 'none']:
        reg.update(2.0, 2.0, c, 0.9, (0, 0), 0.0)
    assert reg.people['P1'].colour == 'none'


def test_waypoint_is_standoff_towards_observer_and_faces_person():
    reg = PersonRegistry(RegistryConfig(standoff=1.5))
    feed(reg, 10.0, 0.0, noise=0.0, observer=(0.0, 0.0))
    p = reg.confirmed()[0]
    x, y, yaw = reg.waypoint(p)
    assert math.isclose(x, 8.5, abs_tol=1e-6) and math.isclose(y, 0.0, abs_tol=1e-6)
    assert math.isclose(abs(yaw), 0.0, abs_tol=1e-6)      # facing +x, towards the person
    # approaching from the north puts the waypoint north of the person, facing south
    x, y, yaw = reg.waypoint(p, approach_from=(10.0, 20.0))
    assert math.isclose(x, 10.0, abs_tol=1e-6) and math.isclose(y, 1.5, abs_tol=1e-6)
    assert math.isclose(yaw, -math.pi / 2, abs_tol=1e-6)


def test_waypoint_when_closer_than_standoff_stays_put():
    reg = PersonRegistry()
    feed(reg, 1.0, 0.0, noise=0.0, observer=(0.0, 0.0))
    x, y, _ = reg.waypoint(reg.confirmed()[0], approach_from=(0.0, 0.0))
    assert math.isclose(x, 0.0, abs_tol=1e-6)


def test_first_seen_is_kept_from_the_first_sighting():
    reg = PersonRegistry()
    reg.update(1.0, 1.0, 'none', 0.9, (0, 0), 10.0)
    first_wall = reg.people['P1'].wall_first_seen
    reg.update(1.1, 1.0, 'none', 0.9, (0, 0), 20.0)
    p = reg.people['P1']
    assert p.first_seen == 10.0 and p.last_seen == 20.0 and p.wall_first_seen == first_wall > 0


def test_last_seen_advances_while_first_seen_stays():
    reg = PersonRegistry()
    reg.update(1.0, 1.0, 'none', 0.9, (0, 0), 10.0)
    first_wall_last = reg.people['P1'].wall_last_seen
    reg.update(1.0, 1.0, 'none', 0.9, (0, 0), 25.0)
    p = reg.people['P1']
    assert p.first_seen == 10.0 and p.last_seen == 25.0
    assert p.wall_last_seen >= first_wall_last > 0