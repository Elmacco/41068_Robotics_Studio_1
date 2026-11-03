"""Probabilistic roadmap (PRM) over the terrain map, for ranger routes.

Not an executable node: terrain_mapper.py builds it whenever the map changes
and answers plan_ranger_route from it.

This is the 49274 A2 PRM (Tasks 6-8 and 10), adapted to a map with trails:

  - Trail nodes: points every trail_node_spacing along each detected trail's
    centreline. The centreline is the ridge of the trail's distance
    transform (trail_detector.py), which is A2 Task 10's idea of putting
    nodes where the distance transform peaks. Uniform sampling alone (A2
    Task 7) rarely lands on a 1.5 m wide trail: in simple_trees the trail is
    ~13% of the area, and on a test map the routes it gave cost 1.5-2x more
    because they cut across the grass.
  - Grass nodes: A2 Task 7 rejection sampling, drawing random cells until
    grass_nodes of them are free, so a route can leave the trail to reach a
    person off it. The seed is fixed, so a map always gives the same graph.
  - Edges: as in A2, every pair of nodes closer than max_edge_length whose
    straight line crosses no blocked cell.
  - Edge cost: A2 Task 6 energy, |(mu m g cos(theta) + m g sin(theta)) dx|,
    with theta from the two nodes' heights and mu averaged over the cells
    the edge crosses (route_planner.walking_grids), so an edge that cuts a
    corner across grass pays the grass rate for that part. The cost depends
    on direction (uphill costs more), so each edge stores both.
  - Search: Dijkstra (A2's A* with heuristic weight 0, see route_planner.py)
    from the start, with start and goal joined to the graph like any other
    node. The route follows parent back-pointers from the goal (A2 Task 4).

Why a roadmap rather than route_planner's grid search: 50 x 50 m at 0.1 m is
250,000 cells, and a grid route took 0.5-1 s in Python on simple_trees; the
roadmap has about 500 nodes, is built in about 0.3 s and answers a route in
3-5 ms, for routes of the same or slightly lower cost (its edges run at any
angle, not only the grid's 8 directions). It is also small enough to draw on
the dashboard.

No smoothing (A2 Task 5): the edges are already straight lines; the smoother
is there for the grid search's staircase.
"""

import heapq
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from grid_tools import nearest_cell, path_length, resample
from route_planner import Route, RouteConfig, WalkingGrids


@dataclass
class GraphConfig:
    # Spacing of the nodes along each trail's centreline (m). Close enough
    # that the straight edges between them follow the trail's bends.
    trail_node_spacing: float = 1.0

    # Random nodes for off-trail ground (A2's prm_num_nodes). 300 on the
    # 50 x 50 m simple_trees is one per ~8 m2, ~3 m apart.
    grass_nodes: int = 300

    # Longest edge (m), A2's prm_max_edge_length. With 300 grass nodes on
    # simple_trees, 3 m left them in 37 separate pieces; 4 m joins 99% of
    # them into one, so a route can reach anywhere before any trail is
    # mapped. Each edge is costed cell by cell, so a longer one is no less
    # accurate, only slower to check.
    max_edge_length: float = 4.0

    # Random seed for the grass nodes, so the same map gives the same graph.
    seed: int = 0


@dataclass
class TrailGraph:
    nodes: np.ndarray                          # (N, 2) world x, y
    on_trail: np.ndarray                       # (N,) bool: a trail node
    neighbours: List[List[int]]                # A2's node.neighbours
    neighbour_costs: List[List[float]]         # A2's node.neighbour_costs (J)

    def edge_count(self) -> int:
        return sum(len(n) for n in self.neighbours) // 2

    def to_dict(self) -> Dict:
        """Nodes and undirected edges, for the dashboard."""
        edges = [[i, j] for i, nbrs in enumerate(self.neighbours) for j in nbrs if i < j]
        return {'nodes': [[round(float(x), 2), round(float(y), 2), bool(t)]
                          for (x, y), t in zip(self.nodes, self.on_trail)],
                'edges': edges}


class _Edges:
    """A2's edge rule and Task 6 cost on the walking grids."""

    def __init__(self, grids: WalkingGrids, elevation: np.ndarray, resolution: float,
                 origin_x: float, origin_y: float, route_config: RouteConfig):
        self.grids = grids
        self.elevation = elevation
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.mg = route_config.ranger_mass * route_config.gravity

    def cells(self, x, y):
        """Row and column of world points, clipped to the grid."""
        height, width = self.grids.blocked.shape
        row = np.floor((np.asarray(y) - self.origin_y) / self.resolution).astype(np.int64)
        col = np.floor((np.asarray(x) - self.origin_x) / self.resolution).astype(np.int64)
        return np.clip(row, 0, height - 1), np.clip(col, 0, width - 1)

    def height(self, x: float, y: float) -> float:
        row, col = self.cells(x, y)
        z = self.elevation[row, col]
        return float(z) if np.isfinite(z) else math.nan

    def costs(self, a: Sequence[float], b: Sequence[float]) -> Optional[Tuple[float, float]]:
        """(cost a -> b, cost b -> a), or None if the edge crosses a blocked cell."""
        horizontal = math.hypot(b[0] - a[0], b[1] - a[1])
        # Sample every half cell, so the line cannot step over a blocked cell
        samples = max(2, int(math.ceil(horizontal / (0.5 * self.resolution))) + 1)
        rows, cols = self.cells(np.linspace(a[0], b[0], samples),
                                np.linspace(a[1], b[1], samples))
        if self.grids.blocked[rows, cols].any():
            return None
        mu = float(self.grids.mu[rows, cols].mean())

        rise = self.height(*b) - self.height(*a)
        if rise != rise:                     # NaN: unknown height, treat as flat
            rise = 0.0
        theta = math.atan2(rise, horizontal)
        dx = math.hypot(horizontal, rise)
        # A2 Task 6; going back down the same edge is -theta
        forward = abs((mu * self.mg * math.cos(theta) + self.mg * math.sin(theta)) * dx)
        backward = abs((mu * self.mg * math.cos(theta) - self.mg * math.sin(theta)) * dx)
        return forward, backward


def build_graph(grids: WalkingGrids, elevation: np.ndarray, resolution: float,
                origin_x: float, origin_y: float, trails: Sequence,
                config: GraphConfig, route_config: RouteConfig) -> TrailGraph:
    """Roadmap from the detected trails (trail_detector.Trail) and the grids."""
    edges = _Edges(grids, elevation, resolution, origin_x, origin_y, route_config)
    blocked = grids.blocked
    height, width = blocked.shape

    # Trail nodes along each centreline, skipping any that landed on a
    # blocked cell (a trail running past a tree within the clearance)
    points: List[Tuple[float, float]] = []
    for trail in trails:
        for x, y in resample(list(trail.points), config.trail_node_spacing):
            row, col = edges.cells(x, y)
            if not blocked[row, col]:
                points.append((x, y))
    trail_count = len(points)

    # A2 Task 7: rejection sampling. Draw in batches; give up after a
    # generous number of draws in case almost everything is blocked.
    rng = np.random.default_rng(config.seed)
    draws = 0
    grass = 0
    while grass < config.grass_nodes and draws < 50 * max(config.grass_nodes, 1):
        rows = rng.integers(0, height, config.grass_nodes)
        cols = rng.integers(0, width, config.grass_nodes)
        draws += config.grass_nodes
        for row, col in zip(rows, cols):
            if grass == config.grass_nodes:
                break
            if not blocked[row, col]:
                points.append((origin_x + (col + 0.5) * resolution,
                               origin_y + (row + 0.5) * resolution))
                grass += 1

    nodes = np.array(points, dtype=np.float64).reshape(-1, 2)
    on_trail = np.arange(len(nodes)) < trail_count
    neighbours: List[List[int]] = [[] for _ in range(len(nodes))]
    neighbour_costs: List[List[float]] = [[] for _ in range(len(nodes))]

    # Edges between every pair closer than max_edge_length. The pairwise
    # distances are one numpy operation instead of A2's double loop.
    if len(nodes) > 1:
        diff = nodes[:, None, :] - nodes[None, :, :]
        close = np.hypot(diff[..., 0], diff[..., 1]) < config.max_edge_length
        for i, j in zip(*np.nonzero(np.triu(close, k=1))):
            cost = edges.costs(nodes[i], nodes[j])
            if cost is None:
                continue
            neighbours[i].append(int(j))
            neighbour_costs[i].append(cost[0])
            neighbours[j].append(int(i))
            neighbour_costs[j].append(cost[1])

    return TrailGraph(nodes, on_trail, neighbours, neighbour_costs)


def plan_route_prm(graph: TrailGraph, grids: WalkingGrids, elevation: np.ndarray,
                   resolution: float, origin_x: float, origin_y: float,
                   start_xy: Tuple[float, float], goal_xy: Tuple[float, float],
                   config: GraphConfig, route_config: RouteConfig) -> Optional[Route]:
    """Least-energy route from start_xy to goal_xy (world metres) over the roadmap."""
    edges = _Edges(grids, elevation, resolution, origin_x, origin_y, route_config)
    free = ~grids.blocked
    snap = max(1, int(round(route_config.snap_radius / resolution)))

    def snapped(xy):
        """xy if it is on a free cell, else the nearest free cell centre within
        snap_radius, else None (as in plan_route, also for points off the map)."""
        row = int(math.floor((xy[1] - origin_y) / resolution))
        col = int(math.floor((xy[0] - origin_x) / resolution))
        height, width = free.shape
        if 0 <= row < height and 0 <= col < width and free[row, col]:
            return float(xy[0]), float(xy[1])
        cell = nearest_cell(free, row, col, snap)
        if cell is None:
            return None
        return origin_x + (cell[1] + 0.5) * resolution, origin_y + (cell[0] + 0.5) * resolution

    start, goal = snapped(start_xy), snapped(goal_xy)
    if start is None or goal is None:
        return None

    # Start and goal are two extra nodes, N and N + 1, joined to the graph
    # nodes within max_edge_length, or twice that if none are (a person found
    # in a corner the random nodes missed).
    count = len(graph.nodes)
    start_idx, goal_idx = count, count + 1

    def links(xy):
        for reach in (config.max_edge_length, 2.0 * config.max_edge_length):
            if count:
                near = np.nonzero(np.hypot(*(graph.nodes - xy).T) < reach)[0]
            else:
                near = []
            found = []
            for j in near:
                cost = edges.costs(xy, graph.nodes[j])
                if cost is not None:
                    found.append((int(j), cost))
            if found:
                return found
        return []

    start_out = [(j, cost[0]) for j, cost in links(start)]
    goal_in = {j: cost[1] for j, cost in links(goal)}    # cost from node j to the goal
    direct = edges.costs(start, goal) if math.dist(start, goal) < config.max_edge_length else None
    if direct is not None:
        start_out.append((goal_idx, direct[0]))

    def neighbours_of(node):
        if node == start_idx:
            return start_out
        if node == goal_idx:
            return []
        out = list(zip(graph.neighbours[node], graph.neighbour_costs[node]))
        if node in goal_in:
            out.append((goal_idx, goal_in[node]))
        return out

    # Dijkstra, with A2's names. A node can be pushed more than once; stale
    # copies are skipped when popped (see route_planner.py).
    cost_to_node = [math.inf] * (count + 2)
    parent_node = [-1] * (count + 2)
    visited = [False] * (count + 2)
    cost_to_node[start_idx] = 0.0
    unvisited = [(0.0, start_idx)]
    expanded = 0
    while unvisited:
        cost, node = heapq.heappop(unvisited)
        if visited[node]:
            continue
        visited[node] = True
        expanded += 1
        if node == goal_idx:
            break
        for neighbour, edge_cost in neighbours_of(node):
            if visited[neighbour]:
                continue
            if cost + edge_cost < cost_to_node[neighbour]:
                cost_to_node[neighbour] = cost + edge_cost
                parent_node[neighbour] = node
                heapq.heappush(unvisited, (cost + edge_cost, neighbour))

    if not visited[goal_idx]:
        return None

    # A2 Task 4: follow the parents back from the goal, then reverse
    chain = []
    node = goal_idx
    while node != -1:
        chain.append(node)
        node = parent_node[node]
    chain.reverse()
    position = {start_idx: start, goal_idx: goal}
    world = [position[n] if n in position else tuple(graph.nodes[n]) for n in chain]

    points = resample(world, route_config.waypoint_spacing)
    rows, cols = edges.cells([p[0] for p in points], [p[1] for p in points])
    return Route(points=points,
                 length=path_length(world),
                 energy=cost_to_node[goal_idx],
                 trail_share=float(grids.trail[rows, cols].mean()),
                 expanded=expanded)
