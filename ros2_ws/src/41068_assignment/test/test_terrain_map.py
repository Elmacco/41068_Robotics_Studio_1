"""TerrainMap on synthetic point clouds: flat ground, a box, a trail stripe."""
import numpy as np

from terrain_map import TerrainConfig, TerrainMap

GRASS = (60, 140, 50)       # green, hue ~ 54 in OpenCV units
TRAIL = (150, 100, 55)      # brown, hue ~ 15


def ground_points(x0, x1, y0, y1, colour, z=0.0, step=0.025):
    xs, ys = np.meshgrid(np.arange(x0, x1, step), np.arange(y0, y1, step))
    xyz = np.column_stack([xs.ravel(), ys.ravel(), np.full(xs.size, z)])
    rgb = np.tile(np.array(colour, dtype=np.uint8), (len(xyz), 1))
    return xyz, rgb


def box_points(x0, x1, y0, y1, top, step=0.025):
    """Sides and top of a box standing on the ground."""
    pts = []
    for z in np.arange(0.05, top, step):
        for x in np.arange(x0, x1, step):
            pts += [(x, y0, z), (x, y1, z)]
        for y in np.arange(y0, y1, step):
            pts += [(x0, y, z), (x1, y, z)]
    xyz = np.array(pts)
    return xyz, np.tile(np.array((90, 90, 90), np.uint8), (len(xyz), 1))


def make_map():
    return TerrainMap(TerrainConfig(x_min=0.0, x_max=5.0, y_min=0.0, y_max=5.0))


def test_flat_grass_is_observed_free_ground():
    terrain = make_map()
    terrain.add_points(*ground_points(0, 5, 0, 2.5, GRASS))
    occ = terrain.occupancy()
    assert (occ[:24, :] == 0).all()          # rows 0-24 cover y 0-2.5
    assert (occ[26:, :] == -1).all()         # never seen
    assert np.allclose(terrain.elevation()[:24, :], 0.0)
    assert terrain.trail_fraction()[:24, :].max() == 0.0


def test_box_is_an_obstacle_and_ground_around_it_is_not():
    terrain = make_map()
    terrain.add_points(*ground_points(0, 5, 0, 5, GRASS))
    terrain.add_points(*box_points(2.0, 2.5, 2.0, 2.5, top=1.0))
    obstacles = terrain.obstacles()
    assert obstacles[20:25, 20:25].any()     # the box footprint
    assert not obstacles[:15, :].any()       # well away from it
    assert not obstacles[:, :15].any()


def test_low_step_is_not_an_obstacle():
    terrain = make_map()
    terrain.add_points(*ground_points(0, 5, 0, 5, GRASS))
    terrain.add_points(*box_points(2.0, 2.5, 2.0, 2.5, top=0.2))   # below 0.3 m
    assert not terrain.obstacles()[18:27, 18:27].any()


def test_trail_colour_is_counted():
    terrain = make_map()
    terrain.add_points(*ground_points(0, 5, 0, 5, GRASS))
    terrain.add_points(*ground_points(0, 5, 2.0, 2.6, TRAIL))
    fraction = terrain.trail_fraction()
    assert fraction[21:25, :].min() >= 0.4   # trail rows: about half the votes
    assert fraction[:15, :].max() == 0.0


def test_high_altitude_update_only_adds_colour():
    terrain = make_map()
    terrain.add_points(*ground_points(0, 5, 0, 5, GRASS))
    before = terrain.obstacle_hits.copy(), terrain.ground_z.copy(), terrain.point_count.copy()
    # Seen from above: canopy points at 5 m and trail-coloured ground
    canopy = ground_points(1, 3, 1, 3, GRASS, z=5.0)
    terrain.add_points(*canopy, update_geometry=False)
    terrain.add_points(*ground_points(0, 5, 4.0, 4.5, TRAIL), update_geometry=False)
    assert (terrain.obstacle_hits == before[0]).all()
    assert (terrain.ground_z == before[1]).all()
    assert (terrain.point_count == before[2]).all()
    assert terrain.trail_fraction()[41:44, :].min() > 0.0


def test_steep_terrain_is_an_obstacle():
    terrain = make_map()
    xyz, rgb = ground_points(0, 5, 0, 5, GRASS)
    xyz[:, 2] = np.where(xyz[:, 0] > 2.5, 1.0, 0.0)   # a 1 m cliff at x = 2.5
    terrain.add_points(xyz, rgb)
    blocked = terrain.obstacles()
    # Cell 25 (x 2.5-2.6) still has a ground point at x = 2.5, so the step is
    # between cells 25 and 26 and the central-difference slope is steep in both.
    assert blocked[:, 25:27].all()
    # The top of the cliff within half the ground window (0.5 m) of the edge
    # is 1 m above the local ground, so it is an obstacle too; beyond that,
    # and everywhere below the cliff, it is not.
    assert not blocked[:, :24].any()
    assert not blocked[:, 32:].any()


def test_save_and_load_round_trip(tmp_path):
    terrain = make_map()
    terrain.add_points(*ground_points(0, 5, 0, 5, GRASS))
    path = str(tmp_path / 'terrain.npz')
    terrain.save(path)
    loaded = make_map()
    loaded.load(path)
    assert (loaded.occupancy() == terrain.occupancy()).all()


def test_low_branches_are_overhead_not_obstacles():
    terrain = make_map()
    terrain.add_points(*ground_points(0, 5, 0, 5, GRASS))
    terrain.add_points(*ground_points(2.0, 3.0, 2.0, 3.0, GRASS, z=1.5))   # a branch at 1.5 m
    assert not terrain.obstacles()[20:30, 20:30].any()
    assert terrain.overhead()[21:29, 21:29].all()
    assert (terrain.occupancy()[21:29, 21:29] == 0).all()                 # the drone flies under


def raised_path_map(trail_source='height'):
    """Grass with a solid 1.5 m wide path 0.23 m high along y = 1.75-3.25, as
    simple_trees' boxes: the camera sees its top, never the grass under it."""
    terrain = TerrainMap(TerrainConfig(x_min=0.0, x_max=5.0, y_min=0.0, y_max=5.0,
                                       trail_source=trail_source))
    for _ in range(2):          # second pass: local ground is known everywhere
        terrain.add_points(*ground_points(0, 5, 0, 1.75, GRASS))
        terrain.add_points(*ground_points(0, 5, 3.25, 5, GRASS))
        terrain.add_points(*ground_points(0, 5, 1.75, 3.25, TRAIL, z=0.23))
    return terrain


def test_raised_path_edge_is_a_step_not_a_wall():
    # Between cells the 0.23 m edge is a 49 deg slope, over max_slope_deg,
    # but a person steps up it.
    assert not raised_path_map().obstacles().any()


def test_raised_path_is_found_across_its_whole_width():
    path = raised_path_map().raised_path()
    assert path[18:32, :].all()                # y 1.8-3.2 m: all of it
    assert not path[:17, :].any() and not path[33:, :].any()


def test_raised_path_keeps_its_colour_to_the_edges():
    # Its top is 0.23 m above the local ground near the edge; colour is
    # measured from the cell's own ground, so the edge cells still count.
    fraction = raised_path_map().trail_fraction()
    assert fraction[18:32, :].min() > 0.9


def test_trail_source_picks_the_evidence():
    # Brown grass (simple_trees' tinted plane) fools the colour rule, not the
    # height rule; 'both' needs both.
    for source, expected_grass in (('colour', True), ('height', False), ('both', False)):
        terrain = raised_path_map(source)
        for _ in range(3):      # brown grass, outvoting the green seen before
            terrain.add_points(*ground_points(0, 5, 0, 1.0, TRAIL))
        cells = terrain.trail_cells()
        assert cells[20:30, :].all()
        assert cells[2:8, :].all() == expected_grass


def test_launch_pad_is_not_a_raised_path():
    # simple_trees' launch pad: a 2 m disc 0.12 m high with a 1.5 m tier to
    # 0.17 m, lower than the 0.23 m path
    terrain = TerrainMap(TerrainConfig(x_min=0.0, x_max=8.0, y_min=0.0, y_max=8.0,
                                       trail_source='height'))
    xyz, rgb = ground_points(0, 8, 0, 8, GRASS)
    r = np.hypot(xyz[:, 0] - 4.0, xyz[:, 1] - 4.0)
    xyz[:, 2] = np.where(r <= 1.5, 0.171, np.where(r <= 2.0, 0.12, 0.0))
    for _ in range(2):
        terrain.add_points(xyz, rgb)
    assert not terrain.trail_cells().any()
    assert not terrain.obstacles().any()
