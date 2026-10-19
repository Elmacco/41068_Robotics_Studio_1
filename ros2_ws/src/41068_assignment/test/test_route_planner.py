"""plan_route / walking_grids on small synthetic maps."""
import numpy as np

from route_planner import RouteConfig, plan_route, walking_grids

RES = 0.1
N = 60                                     # 6 x 6 m


def grids(obstacles=None, trail_rows=None, observed=None, overhead=None, config=None):
    config = config or RouteConfig()
    obstacles = np.zeros((N, N), bool) if obstacles is None else obstacles
    overhead = np.zeros((N, N), bool) if overhead is None else overhead
    observed = np.ones((N, N), bool) if observed is None else observed
    fraction = np.zeros((N, N))
    if trail_rows is not None:
        fraction[trail_rows, :] = 1.0
    return walking_grids(obstacles, overhead, observed, fraction, RES, config)


def route(walking, start, goal, elevation=None, config=None):
    elevation = np.zeros((N, N)) if elevation is None else elevation
    return plan_route(walking, elevation, RES, 0.0, 0.0, start, goal, config or RouteConfig())


def test_open_ground_gives_a_straight_route():
    r = route(grids(), (1.0, 3.0), (5.0, 3.0))
    assert r is not None
    assert 3.9 < r.length < 4.3
    assert all(abs(y - 3.0) < 0.2 for _, y in r.points)


def test_route_prefers_a_cheaper_trail_detour():
    # Trail along y = 1.0; start and goal at y = 2.0, 4 m apart. Walking down
    # to the trail and back is ~6 m of mostly trail at half the cost per metre.
    r = route(grids(trail_rows=slice(8, 13)), (1.0, 2.0), (5.0, 2.0))
    assert r.trail_share > 0.5
    assert min(y for _, y in r.points) < 1.4


def test_gap_narrower_than_a_person_is_not_used():
    obstacles = np.zeros((N, N), bool)
    obstacles[:, 30] = True
    obstacles[29:32, 30] = False           # 0.3 m gap: narrower than 2 x 0.3 m
    assert route(grids(obstacles=obstacles), (1.0, 3.0), (5.0, 3.0)) is None
    obstacles[20:40, 30] = False           # 2 m gap
    assert route(grids(obstacles=obstacles), (1.0, 3.0), (5.0, 3.0)) is not None


def test_hill_is_walked_around_when_the_detour_is_cheap():
    # A2 Task 6: a steep-sided 2 m mound between start and goal
    rows, cols = np.mgrid[0:N, 0:N]
    elevation = np.clip(2.0 - 0.12 * np.hypot(rows - 30, cols - 30), 0.0, None)
    r = route(grids(), (0.5, 3.0), (5.5, 3.0), elevation=elevation)
    over_the_top = max(elevation[int(y / RES), int(x / RES)] for x, y in r.points)
    assert over_the_top < 1.0


def test_unobserved_ground_is_avoided():
    observed = np.ones((N, N), bool)
    observed[20:40, 15:45] = False         # unmapped block in the way
    r = route(grids(observed=observed), (1.0, 3.0), (5.0, 3.0))
    assert all(not (2.0 < y < 4.0 and 1.5 < x < 4.5) for x, y in r.points)


def test_low_branches_are_passable_but_cost_more():
    # A 0.5 m hedge of low branches across the map at x = 2.8-3.3 m. Ducking
    # through it costs (0.25 - 0.10) * 0.5 m = 0.075 mg more than open grass.
    overhead = np.zeros((N, N), bool)
    overhead[:, 28:33] = True
    r = route(grids(overhead=overhead), (1.0, 3.0), (5.0, 3.0))
    assert r is not None                     # no way round, so it goes under

    def crossing_y(r):
        return np.mean([y for x, y in r.points if 2.8 <= x <= 3.3])

    # A gap 0.5-1.5 m off the straight line: the detour (~0.12 m extra, 0.012
    # mg) is cheaper than ducking, so the route uses the gap.
    overhead[35:45, 28:33] = False
    assert 3.5 <= crossing_y(route(grids(overhead=overhead), (1.0, 3.0), (5.0, 3.0))) <= 4.5

    # Only a gap 2.5 m away: the detour (~1 m extra) costs more than ducking.
    overhead[35:45, 28:33] = True
    overhead[55:60, 28:33] = False
    assert abs(crossing_y(route(grids(overhead=overhead), (1.0, 3.0), (5.0, 3.0))) - 3.0) < 0.5
