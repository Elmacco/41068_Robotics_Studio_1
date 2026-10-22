#!/usr/bin/env python3
"""Send the drone through a fixed list of waypoints via Nav2.

Unlike basic_autonomy_demo.py (which picks random free-space goals from the
map), this node just drives through a fixed route in order. That's enough for
an early checkpoint: fly a known path near obstacles and let the sensor
topics (camera, lidar, imu) stream data along the way.

The route itself lives in mission_waypoints.py (shared with
publish_waypoints.py, which previews the same route without flying it).
"""

import math
import os
import sys
import time
from typing import List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mission_waypoints as mw  # noqa: E402


class WaypointFollower(Node):

    def __init__(self):
        super().__init__('waypoint_follower')

        self.declare_parameter('robot_name', '')
        self.declare_parameter('navigate_action', 'navigate_to_pose')
        # 'default' = the original demo route; 'person_approach' = fly up to
        # person1 and face them (YOLO test); 'line_inspection' = fly out, then
        # turn on the spot to face the red / normal / green line of people. An
        # explicit `waypoints` parameter overrides any of them.
        self.declare_parameter('route', 'default')
        self.declare_parameter('waypoints', [])
        # If > 0, tighten Nav2's goal yaw tolerance (radians) at startup so the
        # drone really ends up facing the waypoint yaw. The shared Nav2 config
        # uses 1.57 rad, which is too loose to guarantee facing the person.
        # Left at -1, a route may supply its own value (line_inspection does).
        self.declare_parameter('yaw_goal_tolerance', -1.0)
        self.declare_parameter('loop', False)
        # Pause after each waypoint. Left at -1, the route's own pause is used
        # (1.0 s for routes that don't set one).
        self.declare_parameter('goal_pause_seconds', -1.0)

        # Per-route defaults, filled in by _select_waypoints().
        self.route_pause_seconds = 1.0
        self.route_yaw_tolerance = -1.0

        self.robot_name = self._resolve_robot_name()
        self.map_frame = f'{self.robot_name}_map'

        self.waypoints = mw.parse_waypoints(self._select_waypoints())
        self.waypoint_index = 0
        self.goal_active = False
        self.next_goal_time = 0.0
        self.last_waiting_log_time = 0.0

        self.nav_client = ActionClient(
            self, NavigateToPose, self.get_parameter('navigate_action').value,
        )
        self.marker_pub = self.create_publisher(MarkerArray, 'waypoint_markers', 1)
        # controller_server lives in the robot namespace (relative name).
        self.yaw_tolerance_set = False
        self.param_client = self.create_client(SetParameters, 'controller_server/set_parameters')
        self.timer = self.create_timer(1.0, self._tick)
        self._publish_markers()

        self.get_logger().info(
            f'Waypoint follower started for "{self.robot_name}" with '
            f'{len(self.waypoints)} waypoints (loop={self.get_parameter("loop").value}).'
        )

    def _select_waypoints(self):
        explicit = list(self.get_parameter('waypoints').value)
        if explicit:
            return explicit
        route = str(self.get_parameter('route').value)
        if route == 'person_approach':
            return mw.PERSON_APPROACH_WAYPOINTS
        if route == 'line_inspection':
            self.route_pause_seconds = mw.LINE_PAUSE_SECONDS
            self.route_yaw_tolerance = mw.LINE_YAW_TOLERANCE
            return mw.LINE_INSPECTION_WAYPOINTS
        if route != 'default':
            self.get_logger().warn(f'Unknown route "{route}", using default.')
        return mw.DEFAULT_WAYPOINTS

    def _tighten_yaw_tolerance(self) -> None:
        """Best-effort: lower the goal-checker yaw tolerance on controller_server."""
        tol = float(self.get_parameter('yaw_goal_tolerance').value)
        if tol <= 0.0:
            tol = self.route_yaw_tolerance
        if tol <= 0.0 or self.yaw_tolerance_set:
            return
        if not self.param_client.service_is_ready():
            return
        req = SetParameters.Request()
        for name in ('general_goal_checker.yaw_goal_tolerance', 'FollowPath.yaw_goal_tolerance'):
            req.parameters.append(Parameter(
                name=name,
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=tol),
            ))
        self.param_client.call_async(req)
        self.yaw_tolerance_set = True
        self.get_logger().info(f'Set goal yaw tolerance to {tol:.2f} rad on controller_server.')

    def _resolve_robot_name(self) -> str:
        configured_name = str(self.get_parameter('robot_name').value).strip().strip('/')
        if configured_name:
            return configured_name
        namespace = self.get_namespace().strip('/')
        return namespace or 'parrot1'

    def _tick(self) -> None:
        self._publish_markers()
        self._tighten_yaw_tolerance()

        if self.goal_active or time.monotonic() < self.next_goal_time:
            return

        if not self.nav_client.server_is_ready():
            if not self.nav_client.wait_for_server(timeout_sec=0.1):
                self._log_waiting('Waiting for Nav2 NavigateToPose action server...')
                return

        if self.waypoint_index >= len(self.waypoints):
            if not bool(self.get_parameter('loop').value):
                self._log_waiting('Route complete. Not looping (loop:=false).')
                return
            self.waypoint_index = 0

        self._send_goal(self.waypoints[self.waypoint_index])

    def _log_waiting(self, message: str) -> None:
        now = time.monotonic()
        if now - self.last_waiting_log_time > 5.0:
            self.last_waiting_log_time = now
            self.get_logger().info(message)

    def _send_goal(self, waypoint: Tuple[float, float, float]) -> None:
        x, y, yaw = waypoint

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        qx, qy, qz, qw = self._quaternion_from_yaw(yaw)
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.goal_active = True
        self.get_logger().info(
            f'Sending waypoint {self.waypoint_index + 1}/{len(self.waypoints)}: '
            f'({x:.2f}, {y:.2f}) in frame {self.map_frame}.'
        )
        self._publish_markers()

        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 goal was rejected.')
            self._advance()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future) -> None:
        status = future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f'Waypoint finished with status {status}, continuing.')
        self._advance()

    def _publish_markers(self) -> None:
        markers = mw.build_marker_array(
            self.waypoints, self.map_frame, self.waypoint_index, self.get_clock().now().to_msg(),
        )
        self.marker_pub.publish(markers)

    def _advance(self) -> None:
        self.goal_active = False
        self.waypoint_index += 1
        pause = float(self.get_parameter('goal_pause_seconds').value)
        if pause < 0.0:
            pause = self.route_pause_seconds
        self.next_goal_time = time.monotonic() + pause
        self._publish_markers()

    @staticmethod
    def _quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
        half_yaw = 0.5 * yaw
        return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


def main(args: Optional[List[str]] = None):
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()