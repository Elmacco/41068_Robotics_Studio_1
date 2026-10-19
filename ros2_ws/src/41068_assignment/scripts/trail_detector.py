"""Find hiking trails in the terrain map and turn them into flyable polylines.

Not an executable node: terrain_mapper.py runs it on the latest terrain map.

Pipeline (each step is a standard image-processing operation on the grid):

  1. Trail mask: cells whose ground points were mostly trail-coloured
     (terrain_map.py classifies each point's colour).
  2. Clean the mask with a morphological close (fill pinholes) and open
     (drop specks).
  3. Distance transform of the mask: each trail cell's distance to the
     nearest grass. It rises at 1 m per m from the trail's edge and peaks
     along its middle.
  4. Ridge: trail cells where the distance transform's gradient is near
     flat (|grad d| < 0.6), i.e. the centreline - the same test as AT3's
     cave_geometry.py, and the "locally high distance transform values"
     idea from A2 Task 10. Requiring d >= min_half_width drops anything too
     narrow to be a trail.
  5. Thin the mask to a one-cell-wide skeleton (Zhang-Suen thinning). This
     is the mask's medial axis, connected through the junctions. (Thinning
     the ridge instead breaks the network apart: at a junction the gradient
     is not flat, so the ridge has gaps exactly where trails meet.)
  6. Split the skeleton at junctions into separate trail sections, using the
     crossing number (how many separate branches leave a pixel: 1 at an end,
     2 along a line, 3 or more at a junction).
  7. Keep a section only if most of it lies on the ridge. Thinning grows a
     spur wherever the trail's edge is bumpy; a spur runs from the middle
     out towards the edge, where the distance transform's gradient is ~1,
     so it is off the ridge. Short sections are dropped too.
  8. Order each section's cells end to end, smooth it with A2's
     gradient-descent smoother (kept inside the trail), and resample it to
     evenly spaced waypoints.

The result is a list of Trail objects, longest first, named T1, T2, ...
Passive mode flies along one of them.
"""

import math
from dataclasses import dataclass, field
from typing import Dict, List, Tuple

import cv2
import numpy as np

from grid_tools import distance_to_outside, path_length, resample, smooth_path


@dataclass
class TrailConfig:
    # A cell is trail if at least this share of its ground points were
    # trail-coloured, judged from at least min_ground_points points.
    min_trail_fraction: float = 0.5
    min_ground_points: int = 3

    # Radius (m) of the close/open clean-up. Small next to a ~0.9 m trail.
    clean_radius: float = 0.15

    # Centreline cells must be at least this far from grass (m), so trails
    # narrower than twice this are ignored as noise. The trails in
    # hiking_trail_1 measure ~0.45 m wide.
    min_half_width: float = 0.1

    # |grad d| below this is the centreline (AT3 cave_geometry value). Away
    # from the middle the distance transform rises at 1 m per m.
    ridge_grad_max: float = 0.6

    # A skeleton section is a trail if at least this share of its cells are
    # ridge cells; spurs into the trail's edge score far lower.
    min_ridge_fraction: float = 0.5

    # Sections shorter than this are spurs from bumpy trail edges (m).
    min_trail_length: float = 2.0

    # A2 smoother weights (A2's parameters.yaml values) and the spacing of
    # the waypoints handed to the path follower (m).
    smoothing_alpha: float = 0.1
    smoothing_beta: float = 0.5
    waypoint_spacing: float = 0.5


@dataclass
class Trail:
    name: str                              # 'T1', 'T2', ... longest first
    points: List[Tuple[float, float]]      # world (x, y) waypoints, end to end
    length: float                          # m
    width: float                           # mean width along the centreline (m)

    def to_dict(self) -> Dict:
        return {'name': self.name, 'length': round(self.length, 2),
                'width': round(self.width, 2),
                'points': [[round(x, 3), round(y, 3)] for x, y in self.points]}


@dataclass
class TrailResult:
    trails: List[Trail] = field(default_factory=list)
    # Intermediate layers, kept for RViz and tests.
    mask: np.ndarray = None
    ridge: np.ndarray = None
    skeleton: np.ndarray = None
    junctions: np.ndarray = None


# Offsets of the 8 neighbours in ring order (N, NE, E, SE, S, SW, W, NW),
# as (d_row, d_col). Ring order matters for the crossing number.
RING = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]


def detect_trails(ground_hits: np.ndarray, trail_hits: np.ndarray,
                  resolution: float, origin_x: float, origin_y: float,
                  config: TrailConfig) -> TrailResult:
    """Run the pipeline above on a terrain map's ground/trail point totals."""
    cfg = config

    # 1. Trail mask
    fraction = trail_hits / np.maximum(ground_hits, 1)
    mask = (ground_hits >= cfg.min_ground_points) & (fraction >= cfg.min_trail_fraction)

    # 2. Clean-up
    radius = max(1, int(round(cfg.clean_radius / resolution)))
    disc = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * radius + 1, 2 * radius + 1))
    mask_u8 = mask.astype(np.uint8)
    mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_CLOSE, disc)
    mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_OPEN, disc)
    mask = mask_u8 > 0

    # 3-4. Distance transform and its flat-gradient ridge
    dist = distance_to_outside(mask) * resolution
    grad_y, grad_x = np.gradient(dist, resolution)
    ridge = mask & (np.hypot(grad_x, grad_y) < cfg.ridge_grad_max) & (dist >= cfg.min_half_width)

    # 5. One-cell-wide skeleton of the whole mask
    skeleton = cv2.ximgproc.thinning(mask.astype(np.uint8) * 255) > 0

    # 6. Split at junctions. Removing each junction with its 8 neighbours
    # separates the branches cleanly, whatever shape the thinning left there.
    junctions = skeleton & (_crossing_number(skeleton) >= 3)
    cut = cv2.dilate(junctions.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
    sections = skeleton & ~cut
    count, labels = cv2.connectedComponents(sections.astype(np.uint8), connectivity=8)

    # 7-8. Keep sections that follow the ridge; order, smooth and resample them
    trails = []
    for label in range(1, count):
        section = labels == label
        if ridge[section].mean() < cfg.min_ridge_fraction:
            continue
        cells = _order_cells(section)
        if len(cells) < 2:
            continue
        smoothed = smooth_path(cells, cfg.smoothing_alpha, cfg.smoothing_beta, blocked=~mask)
        world = [(origin_x + (c + 0.5) * resolution, origin_y + (r + 0.5) * resolution)
                 for r, c in smoothed]
        length = path_length(world)
        if length < cfg.min_trail_length:
            continue
        # Distance is measured cell centre to cell centre; the edge of the
        # trail is half a cell nearer (as in AT3's clearance).
        rows, cols = zip(*cells)
        width = 2.0 * (float(np.mean(dist[list(rows), list(cols)])) - 0.5 * resolution)
        trails.append(Trail('', resample(world, cfg.waypoint_spacing), length, width))

    trails.sort(key=lambda t: t.length, reverse=True)
    for i, trail in enumerate(trails):
        trail.name = f'T{i + 1}'

    return TrailResult(trails, mask, ridge, skeleton, junctions)


def _crossing_number(skeleton: np.ndarray) -> np.ndarray:
    """Per pixel, how many separate runs of skeleton pixels surround it.

    Walk round the 8 neighbours in ring order and count 0 -> 1 steps: 1 at
    the end of a line, 2 along a line, 3 or more where branches meet.
    Unlike a plain neighbour count, a staircase corner (two touching
    neighbours) still reads as 2, so it is not mistaken for a junction.
    """
    padded = np.pad(skeleton.astype(np.int8), 1)
    height, width = skeleton.shape
    ring = [padded[1 + dr:1 + dr + height, 1 + dc:1 + dc + width] for dr, dc in RING]
    crossings = np.zeros((height, width), dtype=np.int8)
    for i in range(8):
        crossings += (ring[i] == 0) & (ring[(i + 1) % 8] == 1)
    return crossings


def _order_cells(section: np.ndarray) -> List[Tuple[int, int]]:
    """The cells of one skeleton section, in order from one end to the other.

    Starts at an end (a cell with one neighbour) and repeatedly steps to an
    unvisited neighbour, trying the 4-connected ones first so a staircase is
    walked step by step instead of cutting a corner and stranding a cell.
    A closed loop has no end, so it starts anywhere.
    """
    cells = set(zip(*np.nonzero(section)))
    if not cells:
        return []

    def neighbours(cell):
        r, c = cell
        four = [(r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)]
        diag = [(r + 1, c + 1), (r + 1, c - 1), (r - 1, c + 1), (r - 1, c - 1)]
        return [n for n in four + diag if n in cells]

    ends = [cell for cell in cells if len(neighbours(cell)) == 1]
    current = min(ends) if ends else min(cells)     # min() keeps runs repeatable
    ordered = [current]
    visited = {current}
    while True:
        options = [n for n in neighbours(current) if n not in visited]
        if not options:
            break
        current = options[0]
        visited.add(current)
        ordered.append(current)
    return [(int(r), int(c)) for r, c in ordered]


def trail_lengths_summary(trails: List[Trail]) -> str:
    """One-line log summary, e.g. '5 trails: T1 12.3 m, T2 8.1 m, ...'."""
    if not trails:
        return 'no trails'
    listed = ', '.join(f'{t.name} {t.length:.1f} m' for t in trails[:8])
    more = f' (+{len(trails) - 8} more)' if len(trails) > 8 else ''
    return f'{len(trails)} trails: {listed}{more}'


def nearest_end_first(points: List[Tuple[float, float]],
                      x: float, y: float) -> List[Tuple[float, float]]:
    """The trail's waypoints oriented so it starts at the end nearer (x, y)."""
    if not points:
        return []
    to_start = math.hypot(points[0][0] - x, points[0][1] - y)
    to_end = math.hypot(points[-1][0] - x, points[-1][1] - y)
    return list(points) if to_start <= to_end else list(reversed(points))
