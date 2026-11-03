"""build_graph / plan_route_prm on synthetic trail maps."""
import cv2
import numpy as np

from route_planner import RouteConfig, plan_route, walking_grids
from trail_detector import TrailConfig, detect_trails
from trail_graph import GraphConfig, build_graph, plan_route_prm

RES = 0.1
H, W = 100, 200                            # 20 x 10 m


def trail_map(lines, width_cells=15):
    """Trail cells for 1.5 m wide trails drawn between (x, y) points in metres."""
    mask = np.zeros((H, W), np.uint8)
    for (x0, y0), (x1, y1) in lines:
        cv2.line(mask, (int(x0 / RES), int(y0 / RES)), (int(x1 / RES), int(y1 / RES)),
                 1, width_cells)
    return mask > 0


def build(trail, obstacles=None, graph_config=None):
    route_config = RouteConfig()
    obstacles = np.zeros((H, W), bool) if obstacles is None else obstacles
    grids = walking_grids(obstacles, np.zeros((H, W), bool), np.ones((H, W), bool),
                          trail, RES, route_config)
    elevation = np.where(trail, 0.23, 0.0)
    trails = detect_trails(trail, RES, 0.0, 0.0, TrailConfig()).trails
    graph_config = graph_config or GraphConfig(grass_nodes=150)
    graph = build_graph(grids, elevation, RES, 0.0, 0.0, trails, graph_config, route_config)
    return graph, grids, elevation, graph_config, route_config


def prm(built, start, goal):
    graph, grids, elevation, graph_config, route_config = built
    return plan_route_prm(graph, grids, elevation, RES, 0.0, 0.0, start, goal,
                          graph_config, route_config)


# A trail along y = 2 m with a bend up to (18, 8), and a goal off it
BENT = [((1.0, 2.0), (12.0, 2.0)), ((12.0, 2.0), (18.0, 8.0))]


def test_trail_nodes_lie_on_the_trail_and_the_graph_is_repeatable():
    trail = trail_map(BENT)
    graph = build(trail)[0]
    rows = (graph.nodes[graph.on_trail, 1] / RES).astype(int)
    cols = (graph.nodes[graph.on_trail, 0] / RES).astype(int)
    assert graph.on_trail.sum() > 15            # ~19 m of trail, a node per metre
    assert trail[rows, cols].all()
    again = build(trail)[0]
    assert np.array_equal(graph.nodes, again.nodes)
    assert graph.edge_count() == again.edge_count()


def test_route_keeps_to_the_trail_and_costs_about_what_the_grid_search_does():
    trail = trail_map(BENT)
    s = build(trail)
    start, goal = (1.0, 4.0), (17.0, 9.5)       # both on grass, ~2 m off the trail
    route = prm(s, start, goal)
    assert route is not None
    assert route.trail_share > 0.7
    _, grids, elevation, _, route_config = s
    grid_route = plan_route(grids, elevation, RES, 0.0, 0.0, start, goal, route_config)
    assert route.energy < 1.15 * grid_route.energy


def test_trail_nodes_beat_uniform_sampling():
    # A2 Task 7 alone: no trail nodes, the same number of random nodes
    trail = trail_map(BENT)
    start, goal = (1.0, 4.0), (17.0, 9.5)
    with_trail = prm(build(trail), start, goal)
    graph, grids, elevation, _, route_config = build(trail)
    uniform_config = GraphConfig(grass_nodes=len(graph.nodes))
    uniform = build_graph(grids, elevation, RES, 0.0, 0.0, [], uniform_config, route_config)
    uniform_route = plan_route_prm(uniform, grids, elevation, RES, 0.0, 0.0, start, goal,
                                   uniform_config, route_config)
    assert with_trail.energy < uniform_route.energy


def test_route_crosses_the_grass_between_separate_trail_systems():
    # Two trails 3 m apart end to end (simple_trees' gap by the launch pad)
    trail = trail_map([((1.0, 5.0), (8.0, 5.0)), ((11.0, 5.0), (19.0, 5.0))])
    route = prm(build(trail), (1.5, 5.0), (18.5, 5.0))
    assert route is not None
    assert route.trail_share > 0.7
    assert route.length < 19.0                  # straight across, no wander


def test_no_edge_crosses_an_obstacle_and_an_enclosed_goal_has_no_route():
    trail = trail_map(BENT)
    obstacles = np.zeros((H, W), bool)
    cv2.circle(obstacles.view(np.uint8), (150, 50), 12, 1, 2)   # a ring round (15, 5)
    graph, grids, *_ = s = build(trail, obstacles)
    for i, neighbours in enumerate(graph.neighbours):
        for j in neighbours:
            (x0, y0), (x1, y1) = graph.nodes[i], graph.nodes[j]
            for t in np.linspace(0.0, 1.0, 40):
                row = int((y0 + t * (y1 - y0)) / RES)
                col = int((x0 + t * (x1 - x0)) / RES)
                assert not grids.blocked[row, col]
    assert prm(s, (1.0, 2.0), (15.0, 5.0)) is None


def test_a_goal_off_the_map_has_no_route():
    route = prm(build(trail_map(BENT)), (1.0, 2.0), (80.0, 80.0))
    assert route is None
