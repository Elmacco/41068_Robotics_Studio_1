"""OdomHistory: the drone's altitude and steadiness at the time a cloud was taken."""
import math
from types import SimpleNamespace

from terrain_mapper import OdomHistory, tilt_deg


def climb(history, start=0.0, end=10.0, climb_at=5.0, rate=1.0, z0=0.8, dt=0.05):
    """Odometry at 20 Hz: hovering at z0, then climbing at `rate` from climb_at."""
    t = start
    while t <= end + 1e-9:
        history.add(t, 1.0, 2.0, z0 + max(0.0, t - climb_at) * rate, 0.0)
        t += dt


def test_altitude_is_from_the_cloud_time_not_the_latest_message():
    history = OdomHistory()
    climb(history, end=5.5)
    # A cloud from 4 s, processed at 5.5 s, 0.7 m up: it was taken at 0.8 m.
    x, y, z, climb_rate, tilt = history.at(4.0, 0.25, 0.1)
    assert (x, y) == (1.0, 2.0) and math.isclose(z, 0.8) and climb_rate == 0.0


def test_a_cloud_taken_as_the_climb_starts_counts_as_moving():
    """The end of Mapping: Return Home starts climbing on the spot. A cloud
    taken just before the climb, processed after it, must not pass as level
    hovering at the latest height (or at the old one)."""
    history = OdomHistory()
    climb(history, end=8.0)
    _, _, z, climb_rate, _ = history.at(4.9, 0.25, 0.1)
    assert math.isclose(z, 0.8, abs_tol=1e-6) and climb_rate > 0.1
    _, _, z, climb_rate, _ = history.at(6.0, 0.25, 0.1)
    assert math.isclose(z, 1.8, abs_tol=1e-6) and math.isclose(climb_rate, 1.0, rel_tol=0.01)


def test_no_odometry_near_the_cloud_time():
    history = OdomHistory()
    climb(history, end=2.0)
    assert history.at(2.5, 0.25, 0.1) is None


def test_old_samples_are_dropped():
    history = OdomHistory(keep=3.0)
    climb(history, end=10.0)
    assert history.samples[0][0] >= 7.0 - 1e-9


def test_tilt_in_the_window_is_reported():
    history = OdomHistory()
    for i in range(20):
        history.add(i * 0.05, 0.0, 0.0, 0.8, 5.0 if i == 10 else 0.0)
    assert history.at(0.45, 0.25, 0.1)[4] == 5.0
    assert history.at(0.1, 0.25, 0.1)[4] == 0.0


def test_tilt_from_quaternion():
    level = SimpleNamespace(x=0.0, y=0.0, z=math.sin(0.6), w=math.cos(0.6))
    assert tilt_deg(level) < 1e-6
    half = math.radians(10.0) / 2       # 10 deg pitch
    pitched = SimpleNamespace(x=0.0, y=math.sin(half), z=0.0, w=math.cos(half))
    assert math.isclose(tilt_deg(pitched), 10.0, rel_tol=1e-6)
