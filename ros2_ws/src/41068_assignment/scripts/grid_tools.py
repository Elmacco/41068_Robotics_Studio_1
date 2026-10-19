"""Small grid helpers shared by the terrain map, trail detector and route planner.

Not an executable node. Everything works on numpy arrays indexed [row, col],
with rows along +y and columns along +x (the nav_msgs/OccupancyGrid layout).

The algorithms are the ones from 49274 Space Robotics:
  - distance_to_outside: the distance transform used by AT3's cave_geometry.py
  - is_occluded and smooth_path: A2's collision check and gradient-descent
    path smoother (Task 5), working in cell coordinates instead of pixels
"""

import math
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np


def distance_to_outside(mask: np.ndarray) -> np.ndarray:
    """Distance in cells from each mask cell to the nearest non-mask cell.

    This is the distance transform from A2 Task 9, computed by OpenCV's exact
    Euclidean version (A2's two-pass 1 / sqrt(2) chamfer approximates it), as
    in AT3. The grid is padded with a non-mask border, so space beyond the
    edge of the map counts as outside rather than as more of the mask.
    """
    padded = cv2.copyMakeBorder(mask.astype(np.uint8), 1, 1, 1, 1,
                                cv2.BORDER_CONSTANT, value=0)
    dist = cv2.distanceTransform(padded, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
    return dist[1:-1, 1:-1]


def is_occluded(blocked: np.ndarray, p1: Sequence[float], p2: Sequence[float]) -> bool:
    """True if the straight line from p1 to p2 (row, col) crosses a blocked cell.

    A2's is_occluded: step along the line one cell at a time and stop at the
    first hit. Points off the grid count as not blocked, as in A2.
    """
    r1, c1 = float(p1[0]), float(p1[1])
    dr, dc = float(p2[0]) - r1, float(p2[1]) - c1
    length = math.hypot(dr, dc)
    if length == 0.0:
        return False
    dr, dc = dr / length, dc / length

    height, width = blocked.shape
    for i in range(int(length) + 1):
        row = int(round(r1 + dr * i))
        col = int(round(c1 + dc * i))
        if not (0 <= row < height and 0 <= col < width):
            return False
        if blocked[row, col]:
            return True
    return False


def smooth_path(path: List[Tuple[float, float]], alpha: float, beta: float,
                blocked: Optional[np.ndarray] = None,
                epsilon: float = 0.001, max_iterations: int = 500
                ) -> List[Tuple[float, float]]:
    """A2's gradient-descent smoother (Task 5), on (row, col) points.

    Each pass moves every point except the ends towards both its original
    position (weight alpha) and its smoothed neighbours (weight beta):

        s_i <- s_i - (alpha + 2 beta) s_i + alpha p_i + beta s_{i-1} + beta s_{i+1}

    An update is rejected if either adjoining segment would cross a blocked
    cell. Iterates until a pass moves the path by less than epsilon (summed
    squared change), as in A2; max_iterations is a safety stop only.
    """
    if len(path) < 3:
        return list(path)

    original = [tuple(map(float, p)) for p in path]
    smooth = [list(p) for p in original]

    for _ in range(max_iterations):
        total_change = 0.0
        for i in range(1, len(smooth) - 1):
            new = [
                smooth[i][k] - (alpha + 2 * beta) * smooth[i][k]
                + alpha * original[i][k] + beta * smooth[i - 1][k] + beta * smooth[i + 1][k]
                for k in (0, 1)
            ]
            if blocked is not None and (is_occluded(blocked, smooth[i - 1], new)
                                        or is_occluded(blocked, new, smooth[i + 1])):
                continue
            total_change += (new[0] - smooth[i][0]) ** 2 + (new[1] - smooth[i][1]) ** 2
            smooth[i] = new
        if total_change < epsilon:
            break

    return [tuple(p) for p in smooth]


def nearest_cell(allowed: np.ndarray, row: int, col: int,
                 max_radius: int) -> Optional[Tuple[int, int]]:
    """The allowed cell closest to (row, col), searching outward in square rings.

    AT3's _nearest_passable: used when a start or goal lands on a blocked or
    unknown cell (e.g. a goal clicked just inside a tree's footprint).
    """
    height, width = allowed.shape
    if 0 <= row < height and 0 <= col < width and allowed[row, col]:
        return row, col
    for radius in range(1, max_radius + 1):
        best = None
        for d_row in range(-radius, radius + 1):
            for d_col in range(-radius, radius + 1):
                if max(abs(d_row), abs(d_col)) != radius:
                    continue
                r, c = row + d_row, col + d_col
                if 0 <= r < height and 0 <= c < width and allowed[r, c]:
                    dist = d_row * d_row + d_col * d_col
                    if best is None or dist < best[0]:
                        best = (dist, r, c)
        if best is not None:
            return best[1], best[2]
    return None


def resample(points: List[Tuple[float, float]], spacing: float) -> List[Tuple[float, float]]:
    """Points every `spacing` along a polyline, keeping both ends.

    Used to turn a cell-by-cell path into evenly spaced waypoints, which is
    what the pure-pursuit follower and RViz want.
    """
    if len(points) < 2:
        return list(points)
    out = [points[0]]
    carried = 0.0                     # distance since the last output point
    for (x0, y0), (x1, y1) in zip(points[:-1], points[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        pos = spacing - carried       # where the next point falls on this segment
        while pos <= seg:
            t = pos / seg
            out.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
            pos += spacing
        carried = (carried + seg) % spacing if seg > 0 else carried
    if math.hypot(out[-1][0] - points[-1][0], out[-1][1] - points[-1][1]) > 1e-6:
        out.append(points[-1])
    return out


def path_length(points: List[Tuple[float, float]]) -> float:
    return sum(math.hypot(x1 - x0, y1 - y0)
               for (x0, y0), (x1, y1) in zip(points[:-1], points[1:]))
