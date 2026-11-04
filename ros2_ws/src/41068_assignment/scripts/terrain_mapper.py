#!/usr/bin/env python3
"""Build the terrain map from the depth camera, find trails, and plan ranger routes.

Runs in every mission mode, so the map keeps growing whenever the drone flies.

Subscribes
  camera/depth/points  sensor_msgs/PointCloud2  RGB-D cloud (camera frame)
  odometry             nav_msgs/Odometry        Gazebo 3D odometry: altitude
Publishes
  terrain/occupancy    nav_msgs/OccupancyGrid   -1 unobserved, 0 free, 100
                                                drone obstacle; Mapping mode
                                                explores it
  terrain/drone_obstacles nav_msgs/OccupancyGrid 0 free or unobserved, 100
                                                drone obstacle; a layer of
                                                Nav2's costmaps
  terrain/cloud        sensor_msgs/PointCloud2  one point per observed cell at
                                                its ground height, coloured
                                                grass / trail / under branches
                                                / obstacle (the way A2 drew
                                                its terrain)
  trails               std_msgs/String          detected trails as JSON, for
                                                Passive mode and the dashboard
  terrain/trail_markers visualization_msgs/MarkerArray
  terrain/overview     sensor_msgs/Image        top-down picture of the map, +y
                                                up (map_overview.py), for the
                                                dashboard
  terrain/overview_info std_msgs/String         JSON: frame, resolution, origin
                                                and size of that picture, and
                                                the ranger station
  terrain/route_graph  std_msgs/String          JSON: the route roadmap's nodes
                                                and edges (trail_graph.py)
  ranger_route         nav_msgs/Path            the last route planned
  ranger_route/info    std_msgs/String          JSON: its goal, length, energy
                                                and share on trail
Services
  terrain/save         std_srvs/Trigger         save map + trails to disk
  plan_ranger_route    nav_msgs/GetPlan         least-effort walking route; an
                                                empty start frame means "from
                                                the ranger station"

TF is 2D in this stack (see spawn_parrot.launch.py), so a cloud is placed with
TF for x, y and heading and with the odometry for altitude.
"""

import array
import json
import math
import os
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from nav_msgs.srv import GetPlan
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from map_overview import render_overview
from node_params import declare_config
from route_planner import RouteConfig, plan_route, walking_grids
from terrain_map import TerrainConfig, TerrainMap
from trail_detector import TrailConfig, detect_trails, trail_lengths_summary
from trail_graph import GraphConfig, build_graph, plan_route_prm

# Colours for terrain/cloud (RGB)
GRASS_RGB = (70, 150, 60)
TRAIL_RGB = (160, 110, 60)
OVERHEAD_RGB = (30, 80, 30)      # under low branches
OBSTACLE_RGB = (210, 40, 40)


def quaternion_to_matrix(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class TerrainMapper(Node):

    def __init__(self):
        super().__init__('terrain_mapper')

        self.declare_parameter('frame_id', 'parrot1_map')
        self.declare_parameter('odom_frame', 'parrot1_odom')
        # Used to name saved maps, e.g. hiking_trail_1_20261020_101500
        self.declare_parameter('world_name', 'world')
        # Where routes start when the request gives no start: the ranger
        # station, set per world in config/worlds/<world>.yaml.
        self.declare_parameter('ranger_station', [0.0, 0.0])
        # Use every n-th row and column of the 720 x 480 cloud. 2 gives ~86k
        # points per cloud, several per cell even at 8 m range.
        self.declare_parameter('cloud_stride', 2)
        # Ignore points further than this from the camera (m). Far points are
        # sparse and graze the ground at a shallow angle.
        self.declare_parameter('max_range', 8.0)
        # Only update heights and obstacles while the drone is at most this
        # high above the ground under it (m): from above the canopy the
        # camera sees tree tops, not the ground (see TerrainMap.add_points).
        self.declare_parameter('max_mapping_height', 3.0)
        # Skip clouds while climbing or descending faster than this (m/s).
        # Altitude comes from the latest odometry, which only matches the
        # moment the cloud was taken while the height is steady.
        self.declare_parameter('max_climb_rate', 0.1)
        self.declare_parameter('publish_period', 1.0)   # map layers (s)
        self.declare_parameter('trail_period', 5.0)     # trail detection (s)
        self.declare_parameter('map_save_dir', '~/41068_maps')
        # A terrain.npz saved earlier to start from, '' to start empty.
        self.declare_parameter('map_load_file', '')

        self.frame_id = self.get_parameter('frame_id').value
        self.odom_frame = self.get_parameter('odom_frame').value

        self.terrain = TerrainMap(declare_config(self, TerrainConfig(), 'terrain.'))
        self.trail_config = declare_config(self, TrailConfig(), 'trails.')
        self.route_config = declare_config(self, RouteConfig(), 'route.')
        self.graph_config = declare_config(self, GraphConfig(), 'graph.')

        # A missing file only warns: demo.launch.py points at the world's
        # preloaded map in maps/, which may not have been recorded yet.
        load_file = os.path.expanduser(self.get_parameter('map_load_file').value)
        if load_file and not os.path.isfile(load_file):
            self.get_logger().warn(f'No terrain map at {load_file}, starting with a blank map')
        elif load_file:
            self.terrain.load(load_file)
            self.get_logger().info(f'Loaded terrain map {load_file} '
                                   f'({100 * self.terrain.coverage():.0f}% observed)')

        self.odom = None
        self.trail_result = None
        self.clouds_used = 0
        # Route roadmap, rebuilt with the trails when the map has changed
        self.graph = None
        self.graph_clouds = -1           # clouds_used when it was built

        # TF callbacks run in their own reentrant group (inside
        # TransformListener) on the multi-threaded executor, so the cloud
        # callback can wait for a transform. Everything else shares one
        # mutually exclusive group so it never touches the map concurrently.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        group = MutuallyExclusiveCallbackGroup()

        self.create_subscription(PointCloud2, 'camera/depth/points', self.cloud_callback, 2,
                                 callback_group=group)
        self.create_subscription(Odometry, 'odometry', self.odom_callback, 10,
                                 callback_group=group)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.occupancy_pub = self.create_publisher(OccupancyGrid, 'terrain/occupancy', latched)
        self.drone_obstacles_pub = self.create_publisher(OccupancyGrid, 'terrain/drone_obstacles',
                                                         latched)
        self.cloud_pub = self.create_publisher(PointCloud2, 'terrain/cloud', 1)
        self.trails_pub = self.create_publisher(String, 'trails', latched)
        self.trail_marker_pub = self.create_publisher(MarkerArray, 'terrain/trail_markers', latched)
        self.route_pub = self.create_publisher(Path, 'ranger_route', latched)
        self.route_info_pub = self.create_publisher(String, 'ranger_route/info', latched)
        self.overview_pub = self.create_publisher(Image, 'terrain/overview', latched)
        self.overview_info_pub = self.create_publisher(String, 'terrain/overview_info', latched)
        self.graph_pub = self.create_publisher(String, 'terrain/route_graph', latched)

        self.create_service(Trigger, 'terrain/save', self.save_callback, callback_group=group)
        self.create_service(GetPlan, 'plan_ranger_route', self.route_callback,
                            callback_group=group)

        self.create_timer(self.get_parameter('publish_period').value, self.publish_layers,
                          callback_group=group)
        self.create_timer(self.get_parameter('trail_period').value, self.update_trails,
                          callback_group=group)

        cfg = self.terrain.config
        self.get_logger().info(
            f'Mapping x [{cfg.x_min}, {cfg.x_max}] y [{cfg.y_min}, {cfg.y_max}] at '
            f'{cfg.resolution} m ({self.terrain.width} x {self.terrain.height} cells)')

    # ------------------------------------------------------------------ input

    def odom_callback(self, msg: Odometry):
        self.odom = msg

    def cloud_callback(self, msg: PointCloud2):
        if self.odom is None:
            return
        if abs(self.odom.twist.twist.linear.z) > self.get_parameter('max_climb_rate').value:
            return

        # Camera pose in the map at the moment the cloud was taken. SLAM only
        # refreshes map -> odom once per processed scan, so it always lags the
        # camera; tf2's "time travel" lookup takes odom -> camera at the
        # cloud's time and map -> odom at the latest time available.
        try:
            transform = self.tf_buffer.lookup_transform_full(
                self.frame_id, Time(), msg.header.frame_id, Time.from_msg(msg.header.stamp),
                self.odom_frame, timeout=Duration(seconds=0.2))
        except TransformException as error:
            self.get_logger().warn(f'No camera transform: {error}', throttle_duration_sec=5.0)
            return

        xyz, rgb = self.read_cloud(msg)
        in_range = np.linalg.norm(xyz, axis=1) <= self.get_parameter('max_range').value
        xyz, rgb = xyz[in_range], rgb[in_range]

        rotation = quaternion_to_matrix(transform.transform.rotation)
        t = transform.transform.translation
        altitude = self.odom.pose.pose.position.z
        world = xyz @ rotation.T + np.array([t.x, t.y, t.z + altitude])

        # Height above the ground under the drone, if that ground is mapped
        position = self.odom.pose.pose.position
        row, col, inside = self.terrain.world_to_cell(np.array([position.x]),
                                                      np.array([position.y]))
        ground = self.terrain.local_ground()[row[0], col[0]] if inside[0] else np.inf
        height = altitude - (ground if np.isfinite(ground) else 0.0)
        update_geometry = height <= self.get_parameter('max_mapping_height').value

        self.terrain.add_points(world, rgb, update_geometry)
        self.clouds_used += 1

    def read_cloud(self, msg: PointCloud2):
        """(N, 3) camera-frame points and (N, 3) uint8 colours, NaNs removed."""
        stride = self.get_parameter('cloud_stride').value
        data = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z', 'rgb'),
                                              skip_nans=False)
        data = data.reshape(msg.height, msg.width, 4)[::stride, ::stride].reshape(-1, 4)
        data = data[np.isfinite(data[:, :3]).all(axis=1)]
        packed = data[:, 3].copy().view(np.uint32)      # 0x00RRGGBB stored as float bits
        rgb = np.column_stack([(packed >> 16) & 0xFF, (packed >> 8) & 0xFF, packed & 0xFF])
        return data[:, :3].astype(np.float64), rgb.astype(np.uint8)

    # ----------------------------------------------------------------- output

    def grid_msg(self, data: np.ndarray, stamp) -> OccupancyGrid:
        terrain = self.terrain
        grid = OccupancyGrid()
        grid.header.frame_id = self.frame_id
        grid.header.stamp = stamp
        grid.info.resolution = terrain.resolution
        grid.info.width = terrain.width
        grid.info.height = terrain.height
        grid.info.origin.position.x = terrain.origin_x
        grid.info.origin.position.y = terrain.origin_y
        grid.info.origin.orientation.w = 1.0
        # From bytes: a 250k-item list takes ~35 ms to check and convert
        grid.data = array.array('b', data.astype(np.int8).tobytes())
        return grid

    def publish_layers(self):
        stamp = self.get_clock().now().to_msg()
        terrain = self.terrain

        occupancy = terrain.occupancy()
        self.occupancy_pub.publish(self.grid_msg(occupancy, stamp))
        # Nav2 gets the obstacles only. Its camera-blind ground (under and
        # behind the drone) is not unknown to it: as NO_INFORMATION the
        # controller would refuse to fly over it.
        self.drone_obstacles_pub.publish(self.grid_msg(np.maximum(occupancy, 0), stamp))

        # Only RViz shows it, so skip building it when RViz isn't running
        if self.cloud_pub.get_subscription_count() > 0:
            self.cloud_pub.publish(self.terrain_cloud(stamp))
        self.publish_overview(stamp)

        self.get_logger().info(
            f'Observed {100 * terrain.coverage():.0f}% of the world '
            f'({self.clouds_used} clouds used)', throttle_duration_sec=10.0)

    def publish_overview(self, stamp):
        """The dashboard's top-down map picture, and where it sits in the world."""
        terrain = self.terrain
        trail = (self.trail_result.mask if self.trail_result is not None
                 else terrain.trail_cells())
        pixels = render_overview(terrain.observed(), terrain.obstacles(), terrain.overhead(), trail)

        image = Image()
        image.header.frame_id = self.frame_id
        image.header.stamp = stamp
        image.height, image.width = pixels.shape[:2]
        image.encoding = 'rgb8'
        image.step = 3 * image.width
        image.data = pixels.tobytes()
        self.overview_pub.publish(image)

        info = {'frame_id': self.frame_id, 'resolution': terrain.resolution,
                'origin': [terrain.origin_x, terrain.origin_y],
                'width': terrain.width, 'height': terrain.height,
                'ranger_station': list(self.get_parameter('ranger_station').value),
                'coverage': round(terrain.coverage(), 3)}
        self.overview_info_pub.publish(String(data=json.dumps(info)))

    def terrain_cloud(self, stamp) -> PointCloud2:
        """One coloured point per observed cell, at its ground height."""
        terrain = self.terrain
        observed = terrain.observed() & np.isfinite(terrain.ground_z)
        rows, cols = np.nonzero(observed)
        x, y = terrain.cell_to_world(rows, cols)
        colour = np.tile(np.array(GRASS_RGB, dtype=np.uint32), (len(rows), 1))
        colour[terrain.trail_cells()[rows, cols]] = TRAIL_RGB
        colour[terrain.overhead()[rows, cols]] = OVERHEAD_RGB
        colour[terrain.obstacles()[rows, cols]] = OBSTACLE_RGB
        packed = (colour[:, 0] << 16) | (colour[:, 1] << 8) | colour[:, 2]

        points = np.zeros(len(rows), dtype=[('x', np.float32), ('y', np.float32),
                                            ('z', np.float32), ('rgb', np.uint32)])
        points['x'], points['y'], points['z'] = x, y, terrain.ground_z[rows, cols]
        points['rgb'] = packed

        cloud = PointCloud2()
        cloud.header.frame_id = self.frame_id
        cloud.header.stamp = stamp
        cloud.height, cloud.width = 1, len(points)
        # rgb is declared FLOAT32 because that is what RViz's RGB8 colour mode
        # expects; the bits are the packed 0x00RRGGBB integer.
        cloud.fields = [PointField(name=n, offset=o, datatype=PointField.FLOAT32, count=1)
                        for n, o in (('x', 0), ('y', 4), ('z', 8), ('rgb', 12))]
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = 16 * len(points)
        cloud.is_dense = True
        cloud.data = points.tobytes()
        return cloud

    def update_trails(self):
        terrain = self.terrain
        self.trail_result = detect_trails(terrain.trail_cells(), terrain.resolution,
                                          terrain.origin_x, terrain.origin_y, self.trail_config)
        trails = self.trail_result.trails
        message = {'frame_id': self.frame_id, 'trails': [t.to_dict() for t in trails]}
        self.trails_pub.publish(String(data=json.dumps(message)))
        self.trail_marker_pub.publish(self.trail_markers(trails))
        self.get_logger().info(trail_lengths_summary(trails), throttle_duration_sec=30.0)

        if self.clouds_used != self.graph_clouds:
            self.rebuild_graph()

    def walking_grids(self):
        terrain = self.terrain
        return walking_grids(terrain.obstacles(), terrain.overhead(), terrain.observed(),
                             terrain.trail_cells(), terrain.resolution, self.route_config)

    def rebuild_graph(self):
        """Route roadmap from the current trails and map (~0.3 s on simple_trees)."""
        terrain = self.terrain
        trails = self.trail_result.trails if self.trail_result is not None else []
        started = time.perf_counter()
        self.graph = build_graph(self.walking_grids(), terrain.elevation(), terrain.resolution,
                                 terrain.origin_x, terrain.origin_y, trails,
                                 self.graph_config, self.route_config)
        self.graph_clouds = self.clouds_used
        message = {'frame_id': self.frame_id, **self.graph.to_dict()}
        self.graph_pub.publish(String(data=json.dumps(message)))
        self.get_logger().info(
            f'Route roadmap: {len(self.graph.nodes)} nodes ({int(self.graph.on_trail.sum())} '
            f'on trails), {self.graph.edge_count()} edges '
            f'({1000 * (time.perf_counter() - started):.0f} ms)', throttle_duration_sec=30.0)

    def trail_markers(self, trails) -> MarkerArray:
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.frame_id
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for i, trail in enumerate(trails):
            line = Marker()
            line.header.frame_id = self.frame_id
            line.ns, line.id = 'trails', i
            line.type, line.action = Marker.LINE_STRIP, Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.12
            line.color.r, line.color.g, line.color.b, line.color.a = 1.0, 0.6, 0.1, 1.0
            line.points = [Point(x=x, y=y, z=0.15) for x, y in trail.points]
            markers.markers.append(line)

            label = Marker()
            label.header.frame_id = self.frame_id
            label.ns, label.id = 'trail_names', i
            label.type, label.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            middle = trail.points[len(trail.points) // 2]
            label.pose.position = Point(x=middle[0], y=middle[1], z=0.8)
            label.pose.orientation.w = 1.0
            label.scale.z = 0.5
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = trail.name
            markers.markers.append(label)
        return markers

    # --------------------------------------------------------------- services

    def save_callback(self, request, response):
        """Save the terrain map, trails and preview images to a new folder."""
        name = f'{self.get_parameter("world_name").value}_{time.strftime("%Y%m%d_%H%M%S")}'
        folder = os.path.join(os.path.expanduser(self.get_parameter('map_save_dir').value), name)
        os.makedirs(folder, exist_ok=True)

        self.terrain.save(os.path.join(folder, 'terrain.npz'))
        self.update_trails()
        with open(os.path.join(folder, 'trails.json'), 'w') as f:
            json.dump([t.to_dict() for t in self.trail_result.trails], f, indent=1)

        # Previews, flipped so +y is up as in RViz: grey unobserved, white
        # free, black obstacle, brown trail.
        preview = np.full((self.terrain.height, self.terrain.width, 3), 128, np.uint8)
        preview[self.terrain.observed()] = (255, 255, 255)
        preview[self.trail_result.mask] = (60, 110, 160)            # BGR brown
        preview[self.terrain.obstacles()] = (0, 0, 0)
        cv2.imwrite(os.path.join(folder, 'terrain.png'), np.flipud(preview))

        response.success = True
        response.message = folder
        self.get_logger().info(f'Saved terrain map to {folder}')
        return response

    def route_callback(self, request, response):
        terrain = self.terrain
        if request.start.header.frame_id:
            start = (request.start.pose.position.x, request.start.pose.position.y)
        else:
            start = tuple(self.get_parameter('ranger_station').value)
        goal = (request.goal.pose.position.x, request.goal.pose.position.y)

        grids = self.walking_grids()
        planner = self.route_config.planner
        started = time.perf_counter()
        if planner == 'grid':
            route = plan_route(grids, terrain.elevation(), terrain.resolution,
                               terrain.origin_x, terrain.origin_y, start, goal, self.route_config)
        else:
            if self.graph is None:
                self.update_trails()             # trails first: they hold the graph's nodes
            route = plan_route_prm(self.graph, grids, terrain.elevation(), terrain.resolution,
                                   terrain.origin_x, terrain.origin_y, start, goal,
                                   self.graph_config, self.route_config)
            if route is None:
                # The roadmap can miss a route the grid has, e.g. through a
                # gap no random node landed near, so check every cell before
                # reporting there is none
                planner = 'grid (no roadmap route)'
                route = plan_route(grids, terrain.elevation(), terrain.resolution,
                                   terrain.origin_x, terrain.origin_y, start, goal,
                                   self.route_config)
        took = time.perf_counter() - started

        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        if route is None:
            self.get_logger().warn(f'No walking route from {start} to {goal}')
        else:
            elevation = terrain.elevation()
            for x, y in route.points:
                pose = PoseStamped()
                pose.header = path.header
                row, col, inside = terrain.world_to_cell(np.array([x]), np.array([y]))
                z = elevation[row[0], col[0]] if inside[0] else math.nan
                pose.pose.position.x, pose.pose.position.y = x, y
                pose.pose.position.z = (float(z) if np.isfinite(z) else 0.0) + 0.1
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
            self.get_logger().info(f'Ranger route {start} -> {goal} ({planner}): '
                                   f'{route.summary()} ({1000 * took:.0f} ms)')
        self.route_pub.publish(path)

        info = {'start': list(start), 'goal': list(goal), 'found': route is not None,
                'planner': planner, 'time_ms': round(1000 * took, 1)}
        if route is not None:
            info.update(length=round(route.length, 2), energy=round(route.energy, 1),
                        trail_share=round(route.trail_share, 3))
        self.route_info_pub.publish(String(data=json.dumps(info)))

        response.plan = path
        return response


def main():
    rclpy.init()
    node = TerrainMapper()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
