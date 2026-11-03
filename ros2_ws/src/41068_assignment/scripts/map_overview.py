"""Top-down picture of the terrain map, for the dashboard.

Not an executable node: terrain_mapper.py renders it and publishes it on
terrain/overview; the dashboard draws the drone, trails, people and routes
on top of it.

Colours (RGB):
  never seen            light grey
  grass                 green
  under low branches    dark green
  trail                 brown, shaded by its distance transform: dark at
                        the grass edge, light along the centreline, the
                        ridge that trail_detector.py follows
  obstacle              near black

The image is turned so +y is up (row 0 is the top, y_max), like RViz and a
map, whereas the grid's row 0 is y_min.
"""

import numpy as np

from grid_tools import distance_to_outside

UNSEEN_RGB = (214, 214, 208)
GRASS_RGB = (126, 170, 98)
OVERHEAD_RGB = (70, 112, 62)
TRAIL_EDGE_RGB = (139, 94, 52)
TRAIL_CENTRE_RGB = (232, 196, 136)
OBSTACLE_RGB = (40, 44, 40)


def render_overview(observed: np.ndarray, obstacles: np.ndarray, overhead: np.ndarray,
                    trail: np.ndarray) -> np.ndarray:
    """(H, W, 3) uint8 RGB image of the map layers, +y up."""
    image = np.empty(observed.shape + (3,), dtype=np.uint8)
    image[:] = UNSEEN_RGB
    image[observed] = GRASS_RGB
    image[observed & overhead] = OVERHEAD_RGB

    trail = trail & observed
    if trail.any():
        # Distance to the nearest grass, scaled so the widest trail's middle
        # is 1, then blended edge colour -> centre colour
        dist = distance_to_outside(trail)
        shade = np.clip(dist / max(float(dist.max()), 1.0), 0.0, 1.0)[trail][:, None]
        edge = np.array(TRAIL_EDGE_RGB, dtype=np.float64)
        centre = np.array(TRAIL_CENTRE_RGB, dtype=np.float64)
        image[trail] = (edge + shade * (centre - edge)).astype(np.uint8)

    image[obstacles] = OBSTACLE_RGB
    return np.flipud(image).copy()
