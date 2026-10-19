"""pure_pursuit and the grid helpers it relies on."""
import math

import numpy as np
from geometry_msgs.msg import Pose2D

from grid_tools import path_length, resample, smooth_path
from mission_modes import FollowConfig, pure_pursuit


def simulate(path, start, config=FollowConfig(), dt=0.2, steps=400):
    """Integrate pure_pursuit's commands with a unicycle model."""
    p = Pose2D()
    p.x, p.y, p.theta = start
    progress, trace = 0, []
    for _ in range(steps):
        speed, yaw_rate, progress, done = pure_pursuit(p, path, progress, config)
        if done:
            return True, trace
        p.theta += yaw_rate * dt
        p.x += speed * math.cos(p.theta) * dt
        p.y += speed * math.sin(p.theta) * dt
        trace.append((p.x, p.y))
    return False, trace


def test_reaches_the_end_of_a_straight_path():
    path = resample([(0.0, 0.0), (10.0, 0.0)], 0.5)
    done, trace = simulate(path, (0.0, 0.0, 0.0))
    assert done
    assert max(abs(y) for _, y in trace) < 0.05


def test_turns_round_to_a_path_behind_it():
    path = resample([(0.0, 0.0), (-8.0, 0.0)], 0.5)
    done, trace = simulate(path, (0.0, 0.0, 0.0))
    assert done


def test_follows_a_corner_closely():
    path = resample([(0.0, 0.0), (6.0, 0.0), (6.0, 6.0)], 0.5)
    done, trace = simulate(path, (0.0, 0.0, 0.0))
    assert done
    # Cuts the corner by less than the lookahead
    assert min(math.hypot(x - 6.0, y) for x, y in trace) < FollowConfig().lookahead


def test_resample_spacing_and_ends():
    points = resample([(0.0, 0.0), (1.0, 0.0), (1.0, 1.3)], 0.5)
    gaps = [math.hypot(x1 - x0, y1 - y0) for (x0, y0), (x1, y1) in zip(points, points[1:])]
    assert points[0] == (0.0, 0.0) and points[-1] == (1.0, 1.3)
    assert all(g <= 0.5 + 1e-9 for g in gaps)
    assert math.isclose(path_length([(0.0, 0.0), (1.0, 0.0), (1.0, 1.3)]), 2.3)


def test_smoothing_rounds_a_corner_but_avoids_obstacles():
    corner = [(0.0, c) for c in range(10)] + [(r, 9.0) for r in range(1, 10)]
    smoothed = smooth_path(corner, 0.1, 0.5)
    assert smoothed[0] == corner[0] and smoothed[-1] == corner[-1]
    assert smoothed[9] != corner[9]                         # the corner moved
    blocked = np.zeros((12, 12), bool)
    blocked[1:, :9] = True                                  # inside of the corner is a wall
    kept = smooth_path(corner, 0.1, 0.5, blocked=blocked)
    assert all(not blocked[int(round(r)), int(round(c))] for r, c in kept)
