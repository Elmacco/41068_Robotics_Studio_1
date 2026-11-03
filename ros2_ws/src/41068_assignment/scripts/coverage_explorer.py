"""Coverage exploration for Mapping mode: fly to the edge of what the camera has seen.

Not an executable node: mapping_mode.py drives it.

This is the frontier explorer from 49274 AT3 (explorer.py), with the same
pipeline - frontier generation, clustering, selection - and the same scoring,
blacklisting and stand-off logic. What changed is the map it runs on:

  AT3 ran on SLAM's lidar map, where "known" meant "seen by the lidar". Here
  it runs on the terrain map (terrain_map.py) published by terrain_mapper.py,
  where "known" means "seen by the depth camera". The lidar would call open
  ground unknown for ever (SLAM only clears space along beams that hit
  something), and the camera is what actually maps the ground, so a frontier
  is the edge between ground the camera has seen and ground it has not. The
  terrain grid only covers the world's area, so exploration is bounded by
  the world's edges for free.

Other changes from AT3, each for a reason given where it is used:
  - sizes are in metres (AT3 counted cells at 0.3 m; this grid is 0.1 m)
  - goals are capped at max_goal_distance (Nav2's rolling global costmap)
  - stand-off clearances match the Parrot's costmap, not the Husky's
  - the marker frame is a parameter (parrot1_map, not map)
  - exploration_stats reports camera coverage of the world
  - AT3's obstacle_clearance frontier filter is gone: it was off by default
    there, kept only to compare against in the report

Like AT3, the algorithm is free of ROS node dependencies so it can be unit
tested; only the RViz marker helpers touch visualization_msgs.
"""

import math
from collections import deque
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
from geometry_msgs.msg import Point, Pose2D
from visualization_msgs.msg import Marker, MarkerArray

# Occupancy cell semantics, as published by terrain_mapper.py: -1 unobserved,
# 0 observed free ground, 100 obstacle. Thresholds rather than equality tests,
# as in AT3, so the explorer also works on a SLAM-style probability grid.
UNKNOWN_MAX = -1     # value <= this is unknown
FREE_MAX = 25        # 0 <= value <= this is traversable free space
OCCUPIED_MIN = 65    # value >= this is an obstacle


class MapGrid:
    """
    Immutable snapshot of an OccupancyGrid with grid <-> world conversions.

    Unchanged from AT3.
    """

    def __init__(self, data: np.ndarray, resolution: float, origin_x: float, origin_y: float):
        self.data = data                  # shape (height, width), row-major
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.height, self.width = data.shape

    @classmethod
    def from_msg(cls, map_msg) -> "MapGrid":
        """Build a snapshot from a nav_msgs/OccupancyGrid message."""
        data = np.asarray(map_msg.data, dtype=np.int8).reshape(
            map_msg.info.height, map_msg.info.width)
        return cls(data=data, resolution=map_msg.info.resolution,
                   origin_x=map_msg.info.origin.position.x,
                   origin_y=map_msg.info.origin.position.y)

    def grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        """Centre of cell (row, col) in map-frame metres."""
        return (self.origin_x + (col + 0.5) * self.resolution,
                self.origin_y + (row + 0.5) * self.resolution)

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Cell (row, col) containing map-frame point (x, y). May be off-grid."""
        return (int(math.floor((y - self.origin_y) / self.resolution)),
                int(math.floor((x - self.origin_x) / self.resolution)))

    def in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.height and 0 <= col < self.width

    def metres_to_cells(self, metres: float) -> int:
        """Convert a distance in metres to a whole number of cells (>= 0)."""
        return max(0, int(math.ceil(metres / self.resolution)))


class Frontier:
    """A connected cluster of frontier cells, i.e. one candidate destination."""

    def __init__(self, frontier_id: int, centroid: Tuple[float, float], size: int):
        self.frontier_id = frontier_id
        self.x, self.y = centroid
        self.size = size                   # number of frontier cells
        self.score = 0.0                   # assigned by the ranking stage

    def distance_to(self, x: float, y: float) -> float:
        """Straight-line distance from a map-frame point to this centroid."""
        return math.hypot(self.x - x, self.y - y)


class CoverageExplorer:
    """
    Frontier detection, clustering, selection and visualisation.

    Usage per cycle (as AT3):
        explorer.update_map(MapGrid.from_msg(map_msg))
        frontiers = explorer.find_frontiers()
        goal = explorer.select_goal(robot_pose)
    """

    def __init__(
        self,
        frame_id: str = 'parrot1_map',
        min_frontier_length: float = 0.9,
        min_unknown_area: float = 2.0,
        weight_gain: float = 1.0,
        weight_distance: float = 2.0,
        weight_turn: float = 0.3,
        blacklist_radius: float = 1.5,
        goal_stale_radius: float = 4.0,
        preferred_clearance: float = 1.0,
        min_clearance: float = 0.75,
        standoff_search: float = 3.0,
        max_goal_distance: float = 9.0,
        heading_window: float = 1.2,
    ):
        self.frame_id = frame_id

        # Frontier ranking weights, unchanged from AT3. weight_gain 0 gives
        # "nearest frontier first". weight_distance above weight_gain keeps the
        # drone finishing the area it is in rather than flipping between
        # regions (AT3 measured that flip-flopping with equal weights).
        self.weight_gain = weight_gain
        self.weight_distance = weight_distance
        self.weight_turn = weight_turn

        # Frontiers within this distance of a failed goal are skipped (AT3).
        # A goal can be blacklisted with a wider radius of its own
        # (mapping_mode.py does for goals that brought no new ground).
        self.blacklist_radius = blacklist_radius

        # A goal is abandoned once no frontier remains within this distance of
        # it, i.e. its frontier was seen on the way there (AT3).
        self.goal_stale_radius = goal_stale_radius

        # Goal stand-off, matched to nav2_params_parrot1.yaml as AT3 matched
        # the Husky's: the footprint is 1 x 1 m (inscribed radius 0.5 m,
        # circumscribed 0.71 m) with a 0.6 m inflation radius. A goal closer
        # than the inscribed radius to an obstacle is lethal and Nav2 rejects
        # it; min_clearance 0.75 sits just past the circumscribed radius, and
        # preferred_clearance 1.0 leaves room to turn on arrival.
        self.preferred_clearance = preferred_clearance
        self.min_clearance = min_clearance
        self.standoff_search = standoff_search

        # AT3 filtered clusters by cell count (3 cells at 0.3 m = 0.9 m of
        # boundary). Expressed in metres here so it survives a resolution
        # change.
        self.min_frontier_length = min_frontier_length

        # Smallest unknown region behind a frontier worth a trip (m^2), as in
        # AT3. Here it filters the sealed shadows tree trunks cast in the
        # camera's view.
        self.min_unknown_area = min_unknown_area

        # Nav2's global costmap is a 20 x 20 m rolling window centred on the
        # drone (nav2_params_parrot1.yaml), and the planner rejects goals off
        # it. Cap goals at a little under its 10 m half-width.
        self.max_goal_distance = max_goal_distance

        # Neighbourhood used to point the goal at the unknown space (m).
        # AT3 used 4 cells at 0.3 m.
        self.heading_window = heading_window

        self.map_: Optional[MapGrid] = None
        self.frontiers_: List[Frontier] = []
        self.frontier_mask_: Optional[np.ndarray] = None
        self.labels_: Optional[np.ndarray] = None
        self.clearance_: Optional[np.ndarray] = None

        self.scan_count_ = 0
        self.blacklist_: List[Tuple[float, float, Optional[float]]] = []   # x, y, radius
        self.selected_: Optional[Frontier] = None
        self.goal_pose_: Optional[Pose2D] = None
        self.goal_cell_xy_: Optional[Tuple[float, float]] = None

    def update_map(self, grid: MapGrid):
        """Store the latest map snapshot."""
        self.map_ = grid

    @property
    def has_map(self) -> bool:
        return self.map_ is not None

    @property
    def has_scanned(self) -> bool:
        """True once frontier detection has run at least once."""
        return self.scan_count_ > 0

    # ------------------------------------------------------------------
    # Frontier detection (AT3)
    # ------------------------------------------------------------------

    def find_frontiers(self) -> List[Frontier]:
        """Detect and cluster frontiers in the current map, largest first."""
        self.frontiers_ = []
        self.frontier_mask_ = None
        self.labels_ = None
        self.clearance_ = None

        if self.map_ is None:
            return []

        # Counted so callers can tell "not run yet" from "nothing left" (AT3).
        self.scan_count_ += 1

        grid = self.map_.data
        unknown = (grid <= UNKNOWN_MAX)
        free = (grid >= 0) & (grid <= FREE_MAX)
        occupied = (grid >= OCCUPIED_MIN)

        # A frontier cell is free space orthogonally adjacent to unknown space.
        cross = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], dtype=np.uint8)
        unknown_grown = cv2.dilate(unknown.astype(np.uint8), cross)
        frontier = free & (unknown_grown > 0)
        self.frontier_mask_ = frontier

        # Distance from every cell to the nearest hazard (m), for the goal
        # stand-off. Unknown counts as a hazard (AT3): unobserved ground is
        # exactly where unmapped trees are. Standing back costs little - the
        # camera still sees the frontier from a metre or two further away.
        hazard = occupied | unknown
        self.clearance_ = cv2.distanceTransform(
            (~hazard).astype(np.uint8), cv2.DIST_L2, 5) * self.map_.resolution

        if not frontier.any():
            return []

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            frontier.astype(np.uint8), connectivity=8)
        self.labels_ = labels

        min_cells = self.map_.metres_to_cells(self.min_frontier_length)
        frontiers: List[Frontier] = []
        for label in range(1, num_labels):          # label 0 is the background
            size = int(stats[label, cv2.CC_STAT_AREA])
            if size < min_cells:
                continue
            col_c, row_c = centroids[label]         # OpenCV gives (x=col, y=row)
            frontiers.append(Frontier(label, self.map_.grid_to_world(row_c, col_c), size))

        if self.min_unknown_area > 0.0:
            frontiers = [f for f in frontiers if self._has_room_behind(f, unknown)]

        frontiers.sort(key=lambda f: f.size, reverse=True)
        self.frontiers_ = frontiers
        return frontiers

    def _has_room_behind(self, frontier: Frontier, unknown: np.ndarray) -> bool:
        """
        Whether a frontier opens into enough unknown space to be worth a trip.

        AT3: flood the unknown region behind the frontier and stop once it is
        big enough. A trunk's shadow is sealed and stops after a few cells;
        unexplored ground runs on.
        """
        cap = int(math.ceil(self.min_unknown_area / (self.map_.resolution ** 2)))
        if cap <= 0:
            return True

        height, width = unknown.shape
        rows, cols = np.nonzero(self.labels_ == frontier.frontier_id)
        seen = set()
        queue = deque()
        for row, col in zip(rows, cols):
            for d_row, d_col in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                n_row, n_col = row + d_row, col + d_col
                if (0 <= n_row < height and 0 <= n_col < width
                        and unknown[n_row, n_col] and (n_row, n_col) not in seen):
                    seen.add((n_row, n_col))
                    queue.append((n_row, n_col))

        while queue:
            if len(seen) >= cap:
                return True
            row, col = queue.popleft()
            for d_row, d_col in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                n_row, n_col = row + d_row, col + d_col
                if (0 <= n_row < height and 0 <= n_col < width
                        and unknown[n_row, n_col] and (n_row, n_col) not in seen):
                    seen.add((n_row, n_col))
                    queue.append((n_row, n_col))

        return len(seen) >= cap

    # ------------------------------------------------------------------
    # Frontier selection (AT3)
    # ------------------------------------------------------------------

    def compute_distance_field(self, start_row: int, start_col: int) -> np.ndarray:
        """
        Breadth-first wavefront from the drone across observed free ground.

        AT3's vectorised wavefront: dilate the reached set one ring at a time.
        Returns travel distance (m) per cell, inf where unreachable. Each ring
        costs one cell width, so diagonal-heavy routes are mild
        underestimates; it is only used to rank candidates against each other.
        """
        grid = self.map_.data
        passable = (grid >= 0) & (grid <= FREE_MAX)
        dist = np.full(grid.shape, np.inf, dtype=np.float32)

        # The drone's own cell can be unobserved (the camera looks ahead, not
        # straight down) or an obstacle, so start from the nearest free cell.
        if not (self.map_.in_bounds(start_row, start_col) and passable[start_row, start_col]):
            found = self._nearest_passable(passable, start_row, start_col)
            if found is None:
                return dist
            start_row, start_col = found

        kernel = np.ones((3, 3), dtype=np.uint8)
        passable_u8 = passable.astype(np.uint8)
        reached = np.zeros(grid.shape, dtype=np.uint8)
        reached[start_row, start_col] = 1
        dist[start_row, start_col] = 0.0

        step = self.map_.resolution
        for ring in range(1, grid.size):
            grown = cv2.dilate(reached, kernel) & passable_u8
            newly = (grown > 0) & (reached == 0)
            if not newly.any():
                break
            dist[newly] = ring * step
            reached = grown
        return dist

    def _nearest_passable(self, passable: np.ndarray, row: int, col: int,
                          max_radius_m: float = 2.0) -> Optional[Tuple[int, int]]:
        """Search outward in rings for the closest passable cell (AT3)."""
        max_radius = self.map_.metres_to_cells(max_radius_m)
        for radius in range(1, max_radius + 1):
            for d_row in range(-radius, radius + 1):
                for d_col in range(-radius, radius + 1):
                    if max(abs(d_row), abs(d_col)) != radius:
                        continue
                    r, c = row + d_row, col + d_col
                    if self.map_.in_bounds(r, c) and passable[r, c]:
                        return r, c
        return None

    def blacklist(self, x: float, y: float, radius: Optional[float] = None):
        """Record a goal the drone failed to reach, so it is not retried.

        Frontier cells within radius of it are skipped; None means
        blacklist_radius.
        """
        self.blacklist_.append((float(x), float(y), None if radius is None else float(radius)))

    def goal_still_useful(self, goal: Optional[Pose2D]) -> bool:
        """Whether a goal being flown to still has unexplored ground at it (AT3)."""
        if goal is None or self.map_ is None:
            return False
        if self.frontier_mask_ is None or not self.frontier_mask_.any():
            return False
        rows, cols = np.nonzero(self.frontier_mask_)
        xs = self.map_.origin_x + (cols + 0.5) * self.map_.resolution
        ys = self.map_.origin_y + (rows + 0.5) * self.map_.resolution
        return bool(np.any(np.hypot(xs - goal.x, ys - goal.y) <= self.goal_stale_radius))

    def clear_selection(self):
        """Forget the current goal, e.g. when another mode takes over."""
        self.selected_ = None
        self.goal_pose_ = None
        self.goal_cell_xy_ = None

    def select_goal(self, robot_pose: Pose2D) -> Optional[Pose2D]:
        """
        Choose the next exploration goal, or None when no frontier is reachable.

        AT3's ranking: information gain (cluster size), travel cost (wavefront
        distance) and turn cost, skipping blacklisted, unreachable and
        out-of-range candidates. The goal faces the unknown ground, which
        matters more here than in AT3: the camera only looks forwards.
        """
        self.clear_selection()
        if self.map_ is None or not self.frontiers_ or robot_pose is None:
            return None

        start_row, start_col = self.map_.world_to_grid(robot_pose.x, robot_pose.y)
        dist_field = self.compute_distance_field(start_row, start_col)

        candidates = []
        for frontier in self.frontiers_:
            cell = self._best_cell_for(frontier, dist_field, robot_pose)
            if cell is not None:
                candidates.append((frontier, *cell))
        if not candidates:
            return None

        max_size = max(f.size for f, _, _, _ in candidates)
        max_dist = max(max(t for _, _, _, t in candidates), 1e-6)

        best = None
        for frontier, row, col, travel in candidates:
            goal_x, goal_y = self.map_.grid_to_world(row, col)
            bearing = math.atan2(goal_y - robot_pose.y, goal_x - robot_pose.x)
            turn = abs(self._angle_difference(bearing, robot_pose.theta))
            frontier.score = (self.weight_gain * (frontier.size / max_size)
                              - self.weight_distance * (travel / max_dist)
                              - self.weight_turn * (turn / math.pi))
            if best is None or frontier.score > best[0].score:
                best = (frontier, row, col)

        frontier, row, col = best

        # Blacklisting must exclude the frontier cell itself, not the
        # stand-off goal, or the same cell is offered again (AT3).
        self.goal_cell_xy_ = self.map_.grid_to_world(row, col)

        row, col = self._standoff_cell(row, col, dist_field)
        goal_x, goal_y = self.map_.grid_to_world(row, col)
        goal = Pose2D()
        goal.x = float(goal_x)
        goal.y = float(goal_y)
        goal.theta = float(self._heading_into_unknown(row, col, robot_pose))

        self.selected_ = frontier
        self.goal_pose_ = goal
        return goal

    def _best_cell_for(self, frontier: Frontier, dist_field: np.ndarray,
                       robot_pose: Pose2D) -> Optional[Tuple[int, int, float]]:
        """
        Pick the reachable cell of a cluster nearest its middle (AT3).

        Cells are excluded per cell, not per cluster, if they are near a
        blacklisted goal (AT3) or beyond max_goal_distance from the drone
        (new: Nav2's rolling costmap), so a big frontier is approached from
        its nearer end rather than abandoned.
        """
        rows, cols = np.nonzero(self.labels_ == frontier.frontier_id)
        if rows.size == 0:
            return None

        costs = dist_field[rows, cols]
        usable = np.isfinite(costs)
        xs = self.map_.origin_x + (cols + 0.5) * self.map_.resolution
        ys = self.map_.origin_y + (rows + 0.5) * self.map_.resolution
        usable &= np.hypot(xs - robot_pose.x, ys - robot_pose.y) <= self.max_goal_distance
        for bx, by, radius in self.blacklist_:
            radius = self.blacklist_radius if radius is None else radius
            usable &= np.hypot(xs - bx, ys - by) > radius
        if not usable.any():
            return None

        centre_row, centre_col = self.map_.world_to_grid(frontier.x, frontier.y)
        to_centre = np.hypot(rows - centre_row, cols - centre_col)
        idx = int(np.argmin(np.where(usable, to_centre, np.inf)))
        return int(rows[idx]), int(cols[idx]), float(costs[idx])

    def _standoff_cell(self, row: int, col: int, dist_field: np.ndarray) -> Tuple[int, int]:
        """
        Shift a goal off the frontier to a cell the drone can occupy (AT3).

        Prefers preferred_clearance, falls back to min_clearance in tight
        spots, and stays within standoff_search of the frontier cell.
        """
        if self.clearance_[row, col] >= self.preferred_clearance:
            return row, col

        height, width = self.clearance_.shape
        radius = self.map_.metres_to_cells(self.standoff_search)
        r0, r1 = max(0, row - radius), min(height, row + radius + 1)
        c0, c1 = max(0, col - radius), min(width, col + radius + 1)

        sub_clear = self.clearance_[r0:r1, c0:c1]
        sub_reach = np.isfinite(dist_field[r0:r1, c0:c1])
        rows_idx, cols_idx = np.mgrid[r0:r1, c0:c1]
        offset = (rows_idx - row) ** 2 + (cols_idx - col) ** 2

        for threshold in (self.preferred_clearance, self.min_clearance):
            usable = sub_reach & (sub_clear >= threshold)
            if usable.any():
                idx = np.argmin(np.where(usable, offset, np.inf))
                flat_row, flat_col = np.unravel_index(idx, offset.shape)
                return int(rows_idx[flat_row, flat_col]), int(cols_idx[flat_row, flat_col])
        return row, col

    def _heading_into_unknown(self, row: int, col: int, robot_pose: Pose2D) -> float:
        """
        Yaw that points the camera at the unknown ground beyond the frontier (AT3).

        Averages the direction to nearby unknown cells; falls back to the
        approach direction.
        """
        grid = self.map_.data
        window = self.map_.metres_to_cells(self.heading_window)
        r0, r1 = max(0, row - window), min(self.map_.height, row + window + 1)
        c0, c1 = max(0, col - window), min(self.map_.width, col + window + 1)
        unknown_rows, unknown_cols = np.nonzero(grid[r0:r1, c0:c1] <= UNKNOWN_MAX)

        if unknown_rows.size > 0:
            d_row = float(unknown_rows.mean()) + r0 - row
            d_col = float(unknown_cols.mean()) + c0 - col
            if abs(d_row) > 1e-9 or abs(d_col) > 1e-9:
                return math.atan2(d_row, d_col)   # rows run along +y, columns +x

        goal_x, goal_y = self.map_.grid_to_world(row, col)
        return math.atan2(goal_y - robot_pose.y, goal_x - robot_pose.x)

    @staticmethod
    def _angle_difference(a: float, b: float) -> float:
        """Smallest signed difference between two angles, in [-pi, pi]."""
        return math.atan2(math.sin(a - b), math.cos(a - b))

    def exploration_stats(self) -> Dict[str, Any]:
        """Coverage summary for logging (AT3's, plus camera coverage)."""
        if self.map_ is None:
            return {'coverage': 0.0, 'observed_area_m2': 0.0, 'num_frontiers': 0}
        grid = self.map_.data
        observed = int((grid >= 0).sum())
        return {
            'coverage': observed / grid.size,
            'observed_area_m2': round(observed * self.map_.resolution ** 2, 1),
            'num_frontiers': len(self.frontiers_),
        }

    # ------------------------------------------------------------------
    # RViz (AT3)
    # ------------------------------------------------------------------

    def create_marker_array(self) -> MarkerArray:
        """Frontier cells (cyan), cluster labels, current goal (green arrow) and
        blacklisted goals (red cubes)."""
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.frame_id
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        if self.map_ is None:
            return markers

        if self.frontier_mask_ is not None and self.frontier_mask_.any():
            cells = self._marker('frontier_cells', 1, Marker.CUBE_LIST, (0.0, 0.9, 1.0, 0.9))
            cells.scale.x = cells.scale.y = self.map_.resolution
            cells.scale.z = 0.05
            for row, col in zip(*np.nonzero(self.frontier_mask_)):
                cells.points.append(self._point(*self.map_.grid_to_world(row, col), 0.1))
            markers.markers.append(cells)

        for frontier in self.frontiers_:
            label = self._marker('frontier_labels', frontier.frontier_id,
                                 Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0, 1.0))
            label.pose.position = self._point(frontier.x, frontier.y, 0.8)
            label.scale.z = 0.4
            label.text = f'F{frontier.frontier_id} n={frontier.size}'
            markers.markers.append(label)

        if self.goal_pose_ is not None:
            arrow = self._marker('frontier_goal', 0, Marker.ARROW, (0.1, 1.0, 0.1, 1.0))
            arrow.pose.position = self._point(self.goal_pose_.x, self.goal_pose_.y, 0.4)
            arrow.pose.orientation.z = math.sin(self.goal_pose_.theta / 2.0)
            arrow.pose.orientation.w = math.cos(self.goal_pose_.theta / 2.0)
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 1.5, 0.2, 0.2
            markers.markers.append(arrow)

        for index, (bx, by, _) in enumerate(self.blacklist_):
            cube = self._marker('frontier_blacklist', index, Marker.CUBE, (1.0, 0.1, 0.1, 0.8))
            cube.pose.position = self._point(bx, by, 0.2)
            cube.scale.x = cube.scale.y = cube.scale.z = 0.4
            markers.markers.append(cube)
        return markers

    def _marker(self, ns: str, marker_id: int, marker_type: int, rgba) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.ns = ns
        marker.id = int(marker_id)
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
        return marker

    @staticmethod
    def _point(x: float, y: float, z: float) -> Point:
        point = Point()
        point.x, point.y, point.z = float(x), float(y), float(z)
        return point
