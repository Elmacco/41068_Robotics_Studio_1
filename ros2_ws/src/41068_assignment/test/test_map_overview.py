"""render_overview colours and orientation."""
import numpy as np

from map_overview import (GRASS_RGB, OBSTACLE_RGB, TRAIL_CENTRE_RGB, TRAIL_EDGE_RGB,
                          UNSEEN_RGB, render_overview)


def test_layers_get_their_colours_and_north_is_up():
    shape = (20, 30)
    observed = np.zeros(shape, bool)
    observed[:10, :] = True                     # the southern half has been seen
    obstacles = np.zeros(shape, bool)
    obstacles[2, 3] = True
    trail = np.zeros(shape, bool)
    trail[4:9, :] = True                        # 5 cells wide, rows 4-8
    image = render_overview(observed, obstacles, np.zeros(shape, bool), trail)

    assert image.shape == (20, 30, 3)
    # Row 0 of the grid (y_min) is the image's bottom row
    assert tuple(image[0, 0]) == UNSEEN_RGB             # north, unseen
    assert tuple(image[-1, 0]) == GRASS_RGB             # south, seen
    assert tuple(image[-1 - 2, 3]) == OBSTACLE_RGB
    # Trail: lightest along its middle (row 6), darker at its edge (row 4)
    middle, edge = image[-1 - 6, 15].astype(int), image[-1 - 4, 15].astype(int)
    assert abs(middle - np.array(TRAIL_CENTRE_RGB)).sum() < 10
    assert abs(edge - np.array(TRAIL_EDGE_RGB)).sum() < abs(middle - np.array(TRAIL_EDGE_RGB)).sum()
