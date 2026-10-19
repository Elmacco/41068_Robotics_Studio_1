"""Plan the least-effort walking route for a ranger across the terrain map.

Not an executable node: terrain_mapper.py serves it as a ROS service.

This is the 49274 A2 path planner on the drone's terrain map:

  - Graph: every cell is a node, joined to its 8 neighbours.
  - Edge cost: A2's energy model (Task 6),
        E = | (mu m g cos(theta) + m g sin(theta)) dx |
    with theta the slope between the two cells (from the terrain map's
    elevation), dx the 3D step length, and mu the terrain's resistance. A2
    notes mu "is a property of the terrain type"; here it is low on trail,
    higher on grass, higher again under low branches, and highest on ground
    the drone never saw.
  - Obstacles: anything a person would walk into, or terrain too steep to
    walk (A2's "large gradient = obstacle"), grown by the person's
    half-width using the distance transform (A2 Task 9), so the route never
    squeezes through a gap a person does not fit.
  - Search: Dijkstra, which A2's A* becomes with heuristic weight 0 - A2
    recommends that for energy costs, because a downhill step can cost less
    than the flat-ground heuristic would assume.
  - Path: follow parent back-pointers from the goal to the start and
    reverse (A2 Task 4), then smooth with A2's smoother (Task 5). The
    smoother only knows about obstacles, so unobserved cells are added to
    its collision check; otherwise it cuts corners into ground the search
    deliberately went around.

The one structural change from A2 is a binary heap for the unvisited set.
A2 scanned the whole unvisited list for the cheapest node, which is fine
for a few hundred PRM nodes but too slow for a 250 x 250 cell grid. A node
can be pushed more than once; the stale copies are skipped when popped,
which has the same effect as A2 lowering a node's cost in place.
"""

import heapq
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from grid_tools import distance_to_outside, nearest_cell, path_length, resample, smooth_path


@dataclass
class RouteConfig:
    # A2 energy model. Mass and gravity scale every edge equally, so they do
    # not change which route wins; they make the reported energy a real
    # figure in joules for a ranger with a pack.
    ranger_mass: float = 90.0        # kg
    gravity: float = 9.81            # m/s^2

    # Resistance per terrain type. Only the ratios change the route: a route
    # along trail is chosen over grass unless the trail is twice as long,
    # and unmapped ground is avoided unless the detour is three times as
    # long - it could hide anything.
    mu_trail: float = 0.05
    mu_grass: float = 0.10
    mu_unknown: float = 0.30
    # Under low branches (1.2-2.0 m): passable by ducking or pushing
    # through, at 2.5x the effort of open grass, trail or not.
    mu_overhead: float = 0.25

    # A cell is trail if at least this share of its ground points were
    # trail-coloured (same rule as trail_detector.py).
    min_trail_fraction: float = 0.5

    # Half a person's width (m). Cells closer than this to an obstacle are
    # blocked, so the route only uses gaps at least 0.6 m wide.
    clearance: float = 0.3

    # A2's heuristic weight. 0 = Dijkstra (see module docstring).
    heuristic_weight: float = 0.0

    # How far a start or goal on a blocked cell may be moved to free ground
    # (m), e.g. a casualty lying right against a tree.
    snap_radius: float = 1.5

    # A2 smoother weights and the output waypoint spacing (m).
    smoothing_alpha: float = 0.1
    smoothing_beta: float = 0.5
    waypoint_spacing: float = 0.5


@dataclass
class WalkingGrids:
    """The terrain map's layers, turned into what the route search needs."""
    blocked: np.ndarray      # cells a person cannot enter
    mu: np.ndarray           # A2 resistance per cell
    trail: np.ndarray        # cells on a trail (only used for reporting)
    unobserved: np.ndarray   # cells the drone never saw (kept out of smoothing)


@dataclass
class Route:
    points: List[Tuple[float, float]]   # world (x, y), start to goal
    length: float                       # horizontal length (m)
    energy: float                       # A2 energy estimate (J)
    trail_share: float                  # fraction of the route's cells on trail
    expanded: int                       # nodes moved to the visited set

    def summary(self) -> str:
        return (f'{self.length:.1f} m, {self.energy / 1000:.1f} kJ, '
                f'{100 * self.trail_share:.0f}% on trail, {self.expanded} cells searched')


# 8-connected neighbours: (d_row, d_col, step length in cells)
NEIGHBOURS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
              (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
              (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]


def walking_grids(obstacles: np.ndarray, overhead: np.ndarray, observed: np.ndarray,
                  trail_fraction: np.ndarray, resolution: float,
                  config: RouteConfig) -> WalkingGrids:
    """Blocked mask, per-cell mu and trail mask from the terrain map's layers."""
    # Clearance from each cell's edge to the nearest obstacle (m), measured
    # as in AT3: cell-centre distance minus half a cell. The map edge counts
    # as an obstacle (the world ends there).
    clearance = (distance_to_outside(~obstacles) - 0.5) * resolution
    blocked = obstacles | (clearance < config.clearance)

    trail = observed & (trail_fraction >= config.min_trail_fraction)
    mu = np.full(obstacles.shape, config.mu_grass, dtype=np.float64)
    mu[trail] = config.mu_trail
    mu[overhead] = config.mu_overhead       # ducking under branches, even on a trail
    mu[~observed] = config.mu_unknown
    return WalkingGrids(blocked, mu, trail, ~observed)


def plan_route(grids: WalkingGrids, elevation: np.ndarray,
               resolution: float, origin_x: float, origin_y: float,
               start_xy: Tuple[float, float], goal_xy: Tuple[float, float],
               config: RouteConfig) -> Optional[Route]:
    """Least-energy route from start_xy to goal_xy (world metres), or None.

    elevation is the ground height per cell (NaN where unknown; a step to or
    from an unknown height is treated as flat).
    """
    blocked = grids.blocked
    height, width = blocked.shape

    def to_cell(x, y):
        return (int(math.floor((y - origin_y) / resolution)),
                int(math.floor((x - origin_x) / resolution)))

    free = ~blocked
    snap = max(1, int(round(config.snap_radius / resolution)))
    start = nearest_cell(free, *to_cell(*start_xy), snap)
    goal = nearest_cell(free, *to_cell(*goal_xy), snap)
    if start is None or goal is None:
        return None

    # Flat Python lists: indexing them in the search loop is far faster than
    # indexing numpy arrays one element at a time.
    blocked_flat = blocked.ravel().tolist()
    mu_flat = grids.mu.ravel().tolist()
    elev_flat = np.where(np.isfinite(elevation), elevation, np.nan).ravel().tolist()
    mg = config.ranger_mass * config.gravity

    start_idx = start[0] * width + start[1]
    goal_idx = goal[0] * width + goal[1]
    goal_r, goal_c = goal

    cost_to_node = [math.inf] * (height * width)
    parent_node = [-1] * (height * width)
    visited = [False] * (height * width)
    cost_to_node[start_idx] = 0.0

    # Heuristic (A2): weighted straight-line distance to the goal, costed at
    # the cheapest flat-ground rate so that with weight 1 it never
    # overestimates on flat terrain.
    h_per_cell = config.heuristic_weight * config.mu_trail * mg * resolution

    unvisited = [(0.0, start_idx)]
    expanded = 0
    while unvisited:
        _, node_idx = heapq.heappop(unvisited)
        if visited[node_idx]:
            continue                     # stale copy of a node already visited
        visited[node_idx] = True
        expanded += 1
        if node_idx == goal_idx:
            break

        row, col = divmod(node_idx, width)
        for d_row, d_col, step in NEIGHBOURS:
            n_row, n_col = row + d_row, col + d_col
            if not (0 <= n_row < height and 0 <= n_col < width):
                continue
            neighbour_idx = n_row * width + n_col
            if blocked_flat[neighbour_idx] or visited[neighbour_idx]:
                continue

            # A2 Task 6 energy cost between the two cells
            horizontal = step * resolution
            rise = elev_flat[neighbour_idx] - elev_flat[node_idx]
            if rise != rise:             # NaN: unknown height, treat as flat
                rise = 0.0
            theta = math.atan2(rise, horizontal)
            dx = math.hypot(horizontal, rise)
            mu_edge = 0.5 * (mu_flat[node_idx] + mu_flat[neighbour_idx])
            edge_cost = abs((mu_edge * mg * math.cos(theta) + mg * math.sin(theta)) * dx)

            cost = cost_to_node[node_idx] + edge_cost
            if cost < cost_to_node[neighbour_idx]:
                cost_to_node[neighbour_idx] = cost
                parent_node[neighbour_idx] = node_idx
                heuristic = h_per_cell * math.hypot(goal_r - n_row, goal_c - n_col)
                heapq.heappush(unvisited, (cost + heuristic, neighbour_idx))

    if not visited[goal_idx]:
        return None                      # unvisited set ran out: no route exists

    # A2 Task 4: follow the parents back from the goal, then reverse
    cells = []
    node_idx = goal_idx
    while node_idx != -1:
        cells.append(divmod(node_idx, width))
        node_idx = parent_node[node_idx]
    cells.reverse()

    on_trail = sum(bool(grids.trail[r, c]) for r, c in cells)
    smoothed = smooth_path(cells, config.smoothing_alpha, config.smoothing_beta,
                           blocked=blocked | grids.unobserved)
    world = [(origin_x + (c + 0.5) * resolution, origin_y + (r + 0.5) * resolution)
             for r, c in smoothed]

    return Route(points=resample(world, config.waypoint_spacing),
                 length=path_length(world),
                 energy=cost_to_node[goal_idx],
                 trail_share=on_trail / len(cells),
                 expanded=expanded)
