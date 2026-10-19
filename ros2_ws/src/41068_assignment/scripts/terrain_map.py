"""2.5D terrain map of the world, built from the drone's RGB-D point cloud.

Not an executable node: terrain_mapper.py feeds it and publishes it.

Why not just use SLAM Toolbox's map: it only marks free space along lidar
beams that hit something, so open ground stays "unknown" for ever, and a
single horizontal lidar plane cannot see the ground's height, anything lower
than the drone, or what the ground looks like (trail or grass). The depth
camera looks at the ground itself, so this map is built from it instead.

Each grid cell keeps running totals over every camera point that landed in it:

  point_count    all points           -> observed: has the camera seen it?
  ground_z       lowest point         -> ground height (the ground is always
                                         the lowest surface in a cell)
  obstacle_hits  points 0.3-1.2 m     -> things a walking person cannot get
                 above local ground      past: trunks, rocks, bushes, people
  overhead_hits  points 1.2-2.0 m     -> low branches: a person can duck
                 above local ground      under or push through, but slowly
  ground_hits    points on the ground
  trail_hits     ... that are trail-coloured (brown) -> trail fraction

From those it derives the layers the rest of the system uses: observed,
obstacles (including terrain too steep to walk, the A2 rule that a large
terrain gradient is an obstacle), overhead branches, elevation, slope and
trail fraction.

Why two height bands: with a single 0.3-2.0 m band, the canopies of the oak
rows in hiking_trail_1 (which start 1.23 m up and are up to 12 m across)
merged into a solid wall that sealed off a quarter of the world, although
the trails run straight under them.

Like AT3's explorer.py, this is plain numpy with no ROS dependencies, so it
can be unit tested without the simulator.
"""

import math
from dataclasses import dataclass
from typing import Tuple

import cv2
import numpy as np


@dataclass
class TerrainConfig:
    # World area to map, in metres in the map frame (= world frame, see
    # spawn_parrot.launch.py). Set per world in config/worlds/<world>.yaml.
    x_min: float = -7.5
    x_max: float = 7.5
    y_min: float = -7.5
    y_max: float = 7.5

    # Cell size (m). 0.1 m resolves a 0.3 m tree trunk and a 0.9 m trail
    # (9 cells across) while keeping a 25 x 25 m world at 250 x 250 cells.
    resolution: float = 0.1

    # A cell counts as observed once this many camera points have landed in
    # it. More than 1 so a single stray point does not count as coverage.
    min_observed_points: int = 3

    # Window (m) for the local ground height: the lowest ground in the
    # surrounding window. A cell where the camera only saw the top of a rock
    # or a person's shoulders has a "lowest point" well above the real
    # ground; taking the minimum over the neighbourhood recovers the ground
    # around it, so the object is measured as standing on it. 1 m covers a
    # rock or person; on a 20 deg slope it underestimates the ground by at
    # most 0.5 m * tan(20 deg) = 0.18 m, still below obstacle_min_height.
    ground_window: float = 1.0

    # Height bands above local ground (m) for a walking person. Below 0.3 m
    # is a step they walk over. 0.3 m to chest height (1.2 m) blocks them.
    # Chest to head height (2.0 m) is low branches they can duck under or
    # push through. Above head height is canopy they walk under.
    obstacle_min_height: float = 0.3
    overhead_min_height: float = 1.2
    head_height: float = 2.0
    min_obstacle_points: int = 3

    # Points within this height of the local ground are ground, and their
    # colour is used for trail detection (m).
    ground_tolerance: float = 0.15

    # Terrain steeper than this is impassable on foot (deg).
    max_slope_deg: float = 35.0

    # Trail colour in OpenCV HSV units (H 0-180, S 0-255). Measured in
    # hiking_trail_1 against the texture: trail hue 12-25 (5th-95th
    # percentile), grass 40-56. The threshold sits between them. The
    # saturation floor keeps grey rock and bark from passing as trail.
    trail_hue_min: int = 5
    trail_hue_max: int = 30
    trail_saturation_min: int = 40


class TerrainMap:
    """Accumulates camera points into the per-cell totals described above."""

    def __init__(self, config: TerrainConfig):
        self.config = config
        res = config.resolution
        self.origin_x = config.x_min
        self.origin_y = config.y_min
        self.width = int(round((config.x_max - config.x_min) / res))
        self.height = int(round((config.y_max - config.y_min) / res))
        shape = (self.height, self.width)

        self.point_count = np.zeros(shape, dtype=np.int32)
        self.ground_z = np.full(shape, np.inf, dtype=np.float32)
        self.obstacle_hits = np.zeros(shape, dtype=np.int32)
        self.overhead_hits = np.zeros(shape, dtype=np.int32)
        self.ground_hits = np.zeros(shape, dtype=np.int32)
        self.trail_hits = np.zeros(shape, dtype=np.int32)

    # ------------------------------------------------------------ coordinates

    @property
    def resolution(self) -> float:
        return self.config.resolution

    def world_to_cell(self, x: np.ndarray, y: np.ndarray
                      ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Row, column and an in-bounds mask for arrays of world points."""
        col = np.floor((np.asarray(x) - self.origin_x) / self.resolution).astype(np.int64)
        row = np.floor((np.asarray(y) - self.origin_y) / self.resolution).astype(np.int64)
        inside = (row >= 0) & (row < self.height) & (col >= 0) & (col < self.width)
        return row, col, inside

    def cell_to_world(self, row, col) -> Tuple[np.ndarray, np.ndarray]:
        """World coordinates of cell centres."""
        x = self.origin_x + (np.asarray(col, dtype=float) + 0.5) * self.resolution
        y = self.origin_y + (np.asarray(row, dtype=float) + 0.5) * self.resolution
        return x, y

    # ---------------------------------------------------------------- update

    def add_points(self, xyz: np.ndarray, rgb: np.ndarray, update_geometry: bool = True):
        """Integrate one point cloud, already in the map frame.

        xyz: (N, 3) float, rgb: (N, 3) uint8.

        update_geometry=False only adds colour votes for points on ground that
        is already mapped. The node uses it when the drone is high up: from
        above the canopy the camera sees tree tops, not the ground under them,
        so heights and obstacles seen from there would be wrong (an oak would
        look like a 12 m wide wall), but the colour of open ground is still
        worth having.
        """
        if len(xyz) == 0:
            return
        x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        row, col, inside = self.world_to_cell(x, y)
        row, col, z, rgb = row[inside], col[inside], z[inside], rgb[inside]
        if len(z) == 0:
            return

        if update_geometry:
            np.add.at(self.point_count, (row, col), 1)
            np.minimum.at(self.ground_z, (row, col), z.astype(np.float32))

        # Height of each point above the ground around it. Cells with no
        # ground yet have local ground +inf, which makes the height -inf and
        # drops the point from every test below.
        height_above = z - self.local_ground()[row, col]

        if update_geometry:
            cfg = self.config
            obstacle = ((height_above >= cfg.obstacle_min_height)
                        & (height_above < cfg.overhead_min_height))
            overhead = ((height_above >= cfg.overhead_min_height)
                        & (height_above <= cfg.head_height))
            np.add.at(self.obstacle_hits, (row[obstacle], col[obstacle]), 1)
            np.add.at(self.overhead_hits, (row[overhead], col[overhead]), 1)

        on_ground = np.abs(height_above) < self.config.ground_tolerance
        trail = self.is_trail_colour(rgb[on_ground])
        np.add.at(self.ground_hits, (row[on_ground], col[on_ground]), 1)
        np.add.at(self.trail_hits, (row[on_ground][trail], col[on_ground][trail]), 1)

    def is_trail_colour(self, rgb: np.ndarray) -> np.ndarray:
        """Boolean per point: is this colour trail (brown) rather than grass?"""
        if len(rgb) == 0:
            return np.zeros(0, dtype=bool)
        hsv = cv2.cvtColor(rgb.reshape(-1, 1, 3).astype(np.uint8),
                           cv2.COLOR_RGB2HSV).reshape(-1, 3)
        cfg = self.config
        return ((hsv[:, 0] >= cfg.trail_hue_min) & (hsv[:, 0] <= cfg.trail_hue_max)
                & (hsv[:, 1] >= cfg.trail_saturation_min))

    # ---------------------------------------------------------------- layers

    def local_ground(self) -> np.ndarray:
        """Lowest ground height within ground_window of each cell (+inf if none).

        A grey-scale erosion is a moving minimum, so this is one OpenCV call.
        The window has an odd number of cells so it is centred on the cell.
        """
        half = int(round(self.config.ground_window / 2 / self.resolution))
        kernel = np.ones((2 * half + 1, 2 * half + 1), dtype=np.uint8)
        return cv2.erode(self.ground_z, kernel, borderType=cv2.BORDER_REPLICATE)

    def observed(self) -> np.ndarray:
        return self.point_count >= self.config.min_observed_points

    def elevation(self) -> np.ndarray:
        """Ground height per cell (m), NaN where the ground was never seen."""
        return np.where(np.isfinite(self.ground_z), self.ground_z, np.nan)

    def slope_deg(self) -> np.ndarray:
        """Terrain slope per cell (deg) from the elevation gradient, NaN if unknown.

        np.gradient takes central differences, so a cell next to unknown
        ground gets NaN rather than a made-up slope.
        """
        grad_y, grad_x = np.gradient(self.elevation(), self.resolution)
        return np.degrees(np.arctan(np.hypot(grad_x, grad_y)))

    def obstacles(self) -> np.ndarray:
        """Cells a walking person cannot enter: something in the way, or too steep."""
        hit = self.obstacle_hits >= self.config.min_obstacle_points
        with np.errstate(invalid='ignore'):
            steep = self.slope_deg() > self.config.max_slope_deg
        return hit | steep

    def overhead(self) -> np.ndarray:
        """Cells with low branches (and nothing lower in the way)."""
        return (self.overhead_hits >= self.config.min_obstacle_points) & ~self.obstacles()

    def trail_fraction(self) -> np.ndarray:
        """Share of each cell's ground points that were trail-coloured (0-1)."""
        return self.trail_hits / np.maximum(self.ground_hits, 1)

    def occupancy(self) -> np.ndarray:
        """OccupancyGrid-style int8: -1 unobserved, 0 free, 100 obstacle.

        Overhead branches count as free: the drone maps at 0.8 m, below them.
        """
        grid = np.full((self.height, self.width), -1, dtype=np.int8)
        grid[self.observed()] = 0
        grid[self.obstacles()] = 100
        return grid

    def coverage(self) -> float:
        """Fraction of the mapped area that has been observed (0-1)."""
        return float(self.observed().mean())

    # ------------------------------------------------------------ save / load

    def save(self, path: str):
        """Write the raw totals to a .npz file (layers are re-derived on load)."""
        cfg = self.config
        np.savez_compressed(
            path,
            bounds=np.array([cfg.x_min, cfg.x_max, cfg.y_min, cfg.y_max]),
            resolution=cfg.resolution,
            point_count=self.point_count, ground_z=self.ground_z,
            obstacle_hits=self.obstacle_hits, overhead_hits=self.overhead_hits,
            ground_hits=self.ground_hits, trail_hits=self.trail_hits)

    def load(self, path: str):
        """Restore totals saved by save(). The grid must match this map's."""
        data = np.load(path)
        cfg = self.config
        same_grid = (np.allclose(data['bounds'], [cfg.x_min, cfg.x_max, cfg.y_min, cfg.y_max])
                     and math.isclose(float(data['resolution']), cfg.resolution))
        if not same_grid:
            raise ValueError(f'{path} was saved with bounds {data["bounds"].tolist()} '
                             f'at {float(data["resolution"])} m, not this map\'s')
        self.point_count = data['point_count']
        self.ground_z = data['ground_z']
        self.obstacle_hits = data['obstacle_hits']
        self.overhead_hits = data['overhead_hits']
        self.ground_hits = data['ground_hits']
        self.trail_hits = data['trail_hits']
