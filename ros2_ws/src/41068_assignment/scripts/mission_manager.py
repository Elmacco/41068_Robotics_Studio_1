#!/usr/bin/env python3
"""Run the drone's mission modes, switched by commands from the dashboard.

Commands arrive as std_msgs/String on mission_command (the web dashboard
already publishes there):

  mapping          explore the world until the camera has seen all of it,
                   save the terrain map, then return home
  passive <trail>  fly along a detected trail at cruise altitude, e.g.
                   'passive T3' (trail names are on the trails topic)
  active           drop to active_altitude over the current spot and hover
  return_home      climb to cruise altitude, fly home, descend
  stop, take_over  stop all autonomy and hover; the dashboard's manual
                   control keeps working (it publishes cmd_vel directly)
  takeoff          hover at mapping altitude (the drone spawns airborne)
  save_map         save the terrain map now

The current mode and what it is doing are published once a second as
std_msgs/String on mission_state, e.g. "MAPPING: 42% observed, flying to F7".

Altitude is set through the flight controller's target_altitude topic;
horizontal motion comes from Nav2 (Mapping mode) or is published here on
cmd_vel (path following). Pose comes from TF (2D: x, y, heading) and
altitude from the Gazebo odometry, as everywhere else in this stack.
"""

import json
import math
import numpy as np

import rclpy
from geometry_msgs.msg import Pose2D, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import Image

from coverage_explorer import CoverageExplorer
from active_mode import ActiveMode
from mapping_mode import MappingMode
from mission_modes import FlyPathMode, FollowConfig, HoverMode
from node_params import declare_config
from trail_detector import nearest_end_first


class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')

        self.declare_parameter('frame_id', 'parrot1_map')
        self.declare_parameter('base_frame', 'parrot1_base_link')
        # Home is where the drone spawned; the launch file passes the spawn pose.
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', -4.0)
        self.declare_parameter('home_z', 0.8)
        # Mapping altitude: the lidar (0.2 m above the drone) then scans at
        # about chest height, seeing trunks and the obstacles a walking
        # person meets, and the camera sees the ground under the canopy.
        self.declare_parameter('mapping_altitude', 0.8)
        # Cruise altitude for Passive mode and Return Home: clear of the
        # tallest tree (oak, 6.5 m) with a 1.5 m margin.
        self.declare_parameter('cruise_altitude', 8.0)
        # Active mode drops to this height for a closer look (m).
        self.declare_parameter('active_altitude', 3.0)
        self.declare_parameter('thermal_max_age', 1.0)
        # The drone counts as at a target altitude within this (m).
        self.declare_parameter('altitude_tolerance', 0.15)
        # Mapping mode goals (see GoalWatchdog in mapping_mode.py). A goal is
        # given up once, for stall_time (s), the camera has seen less than
        # min_gain_area (m2) of new ground and the drone has got less than
        # min_progress (m) closer; 8 s is a spin recovery and a turn on the
        # spot. A goal reached having seen less than min_goal_gain (m2) is
        # treated as failed. Either way its frontier is blacklisted within
        # no_gain_blacklist_radius (m), twice a plain failure's 1.5 m, so the
        # next goal is not the cell beside it with the same view blocked.
        # goal_timeout (s) is only a backstop now: 25 s covers the longest
        # goal (9 m, max_goal_distance) at half Nav2's 1 m/s, plus turning.
        self.declare_parameter('stall_time', 8.0)
        self.declare_parameter('min_gain_area', 0.5)
        self.declare_parameter('min_progress', 0.5)
        self.declare_parameter('min_goal_gain', 1.0)
        self.declare_parameter('no_gain_blacklist_radius', 3.0)
        self.declare_parameter('goal_timeout', 25.0)
        # Finish after this many consecutive selections find no frontier.
        self.declare_parameter('no_goal_limit', 5)
        self.declare_parameter('rate', 5.0)

        self.frame_id = self.get_parameter('frame_id').value
        self.base_frame = self.get_parameter('base_frame').value
        self.home = tuple(self.get_parameter(name).value for name in ('home_x', 'home_y', 'home_z'))
        self.cruise_altitude = self.get_parameter('cruise_altitude').value
        self.altitude_tolerance = self.get_parameter('altitude_tolerance').value
        self.mapping_params = {name: self.get_parameter(name).value for name in (
            'mapping_altitude', 'stall_time', 'min_gain_area', 'min_progress',
            'min_goal_gain', 'no_gain_blacklist_radius', 'goal_timeout', 'no_goal_limit')}
        self.follow_config = declare_config(self, FollowConfig(), 'follow.')
        self.explorer = CoverageExplorer(frame_id=self.frame_id)

        self.altitude = None
        self.terrain_grid = None
        self.trails = {}
        self.latest_thermal = None
        self.latest_thermal_time = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'mission_command', self.command_callback, 10)
        self.create_subscription(Odometry, 'odometry', self.odom_callback, 10)
        self.create_subscription(OccupancyGrid, 'terrain/occupancy', self.grid_callback, latched)
        self.create_subscription(String, 'trails', self.trails_callback, latched)
        # A person detector should publish a map-frame Pose2D here.
        self.create_subscription(Pose2D, 'person_detection', self.person_detection_callback, 10)
        self.create_subscription(Image, 'thermal_camera/image', self.thermal_callback, 10)

        self.state_pub = self.create_publisher(String, 'mission_state', latched)
        self.altitude_pub = self.create_publisher(Float64, 'target_altitude', 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'exploration_markers', 1)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.save_client = self.create_client(Trigger, 'terrain/save')

        self.mode = HoverMode(self, 'IDLE')
        self.status = 'waiting for a command'
        self.create_timer(1.0 / self.get_parameter('rate').value, self.step)
        self.create_timer(1.0, self.publish_state)

    # ------------------------------------------------------------- commands

    def command_callback(self, msg: String):
        words = msg.data.strip().split()
        if not words:
            return
        command, args = words[0].lower(), words[1:]
        home_x, home_y, home_z = self.home

        if command == 'mapping':
            self.switch_to(MappingMode(self, self.explorer))
        elif command == 'passive':
            if not args or args[0].upper() not in self.trails:
                known = ', '.join(sorted(self.trails, key=lambda n: int(n[1:]))) or 'none yet'
                self.get_logger().warn(f'passive needs a trail name; detected trails: {known}')
                return
            pose = self.get_pose()
            if pose is None:
                self.get_logger().warn('No drone pose yet')
                return
            trail = self.trails[args[0].upper()]
            path = nearest_end_first(trail['points'], pose.x, pose.y)
            self.switch_to(FlyPathMode(self, f'PASSIVE {trail["name"]}', path,
                                       self.cruise_altitude))
        elif command == 'active':
            self.switch_to(ActiveMode(self))
        elif command == 'return_home':
            self.switch_to(FlyPathMode(self, 'RETURN HOME', [(home_x, home_y)],
                                       self.cruise_altitude, final_altitude=home_z))
        elif command in ('stop', 'take_over'):
            self.switch_to(HoverMode(self, 'IDLE'))
        elif command == 'takeoff':
            self.switch_to(HoverMode(self, 'IDLE', self.mapping_params['mapping_altitude']))
        elif command == 'save_map':
            self.save_map()
        else:
            self.get_logger().warn(f'Unknown command "{msg.data}"')

    def switch_to(self, mode):
        self.mode.stop()
        self.mode = mode
        self.mode.start()
        self.get_logger().info(f'Mode: {mode.name}')
        # Now rather than at the next 1 s tick, so the dashboard shows it at once.
        self.status = 'starting'
        self.publish_state()

    # ------------------------------------------------------------ main loop

    def step(self):
        self.status = self.mode.step()
        if not self.mode.finished:
            return
        self.get_logger().info(f'{self.mode.name} finished: {self.status}')
        if isinstance(self.mode, MappingMode):
            self.save_map()
            home_x, home_y, home_z = self.home
            self.switch_to(FlyPathMode(self, 'RETURN HOME', [(home_x, home_y)],
                                       self.cruise_altitude, final_altitude=home_z))
        else:
            self.switch_to(HoverMode(self, 'IDLE'))

    def publish_state(self):
        self.state_pub.publish(String(data=f'{self.mode.name}: {self.status}'))

    # --------------------------------------------------- helpers for modes

    def get_pose(self):
        """Drone x, y, heading in the map frame, or None if TF is not ready."""
        try:
            t = self.tf_buffer.lookup_transform(self.frame_id, self.base_frame, Time())
        except TransformException:
            return None
        q = t.transform.rotation
        pose = Pose2D()
        pose.x = t.transform.translation.x
        pose.y = t.transform.translation.y
        pose.theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return pose

    def set_altitude(self, altitude: float):
        self.altitude_pub.publish(Float64(data=float(altitude)))

    def at_altitude(self, altitude: float) -> bool:
        # Re-send the target while waiting, in case the flight controller
        # started after the first message.
        self.altitude_pub.publish(Float64(data=float(altitude)))
        return self.altitude is not None and abs(self.altitude - altitude) < self.altitude_tolerance

    def publish_cmd(self, cmd: Twist):
        self.cmd_pub.publish(cmd)

    def publish_exploration_markers(self, explorer: CoverageExplorer):
        # Built only for RViz: one point per frontier cell, in a Python loop
        if self.marker_pub.get_subscription_count() > 0:
            self.marker_pub.publish(explorer.create_marker_array())

    def save_map(self):
        if not self.save_client.service_is_ready():
            self.get_logger().warn('terrain/save is not available, map not saved')
            return
        future = self.save_client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f: self.get_logger().info(f'Terrain map saved to {f.result().message}'))

    # ------------------------------------------------------------ callbacks

    def odom_callback(self, msg: Odometry):
        self.altitude = msg.pose.pose.position.z

    def grid_callback(self, msg: OccupancyGrid):
        self.terrain_grid = msg

    def trails_callback(self, msg: String):
        data = json.loads(msg.data)
        self.trails = {trail['name']: trail for trail in data['trails']}

    def person_detection_callback(self, pose: Pose2D):
        if isinstance(self.mode, ActiveMode):
            if self.mode.last_person_confirmed:
                return
            self.mode.person_detected(pose)
        elif isinstance(self.mode, MappingMode):
            active_mode = ActiveMode(self)
            if active_mode.person_detected(pose):
                self.get_logger().warning(
                    'Person confirmed during Mapping; interrupting mapping and hovering'
                )
                self.switch_to(active_mode)

    def thermal_callback(self, msg: Image):
        if msg.encoding not in ('16UC1', 'mono16'):
            self.get_logger().warning(
                f'Unexpected thermal image encoding: {msg.encoding}'
            )
            return

        # L16 thermal data contains unsigned 16-bit pixel values.
        pixels = np.frombuffer(msg.data, dtype=np.uint16)

        # msg.step is bytes per row; uint16 uses two bytes per pixel.
        row_pixels = msg.step // 2
        if row_pixels < msg.width:
            self.get_logger().warning(
                f'Thermal image step ({msg.step}) is smaller than its width ({msg.width})'
            )
            return
        required_pixels = msg.height * row_pixels
        if pixels.size < required_pixels:
            self.get_logger().warning(
                f'Thermal image data is truncated: expected {required_pixels} pixels, '
                f'got {pixels.size}'
            )
            return
        pixels = pixels.reshape((msg.height, row_pixels))[:, :msg.width]

        if msg.is_bigendian:
            pixels = pixels.byteswap()

        # The URDF specifies 0.01 Kelvin per pixel unit.
        self.latest_thermal = pixels.astype(np.float32) * 0.01
        self.latest_thermal_time = self.get_clock().now()

    def thermal_is_current(self) -> bool:
        """Return whether the buffered thermal image is recent enough to use."""
        if self.latest_thermal is None or self.latest_thermal_time is None:
            return False
        age = (self.get_clock().now() - self.latest_thermal_time).nanoseconds * 1e-9
        return 0.0 <= age <= self.get_parameter('thermal_max_age').value


def main():
    rclpy.init()
    node = MissionManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
