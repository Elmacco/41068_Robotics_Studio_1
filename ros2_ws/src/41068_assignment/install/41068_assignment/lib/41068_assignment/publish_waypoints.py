#!/usr/bin/env python3
"""Preview the mission route in RViz without flying it.

Publishes the same waypoint markers as waypoint_follower.py (sharing the
route from mission_waypoints.py), but never talks to Nav2. Run this first to
see the planned route, then start waypoint_follower.py separately to fly it.
"""

import os
import sys
from typing import List, Optional

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mission_waypoints as mw  # noqa: E402


class WaypointPreview(Node):

    def __init__(self):
        super().__init__('waypoint_preview')

        self.declare_parameter('robot_name', '')
        self.declare_parameter('waypoints', mw.DEFAULT_WAYPOINTS)

        self.robot_name = self._resolve_robot_name()
        self.map_frame = f'{self.robot_name}_map'
        self.waypoints = mw.parse_waypoints(self.get_parameter('waypoints').value)

        self.marker_pub = self.create_publisher(MarkerArray, 'waypoint_markers', 1)
        self.timer = self.create_timer(1.0, self._publish_markers)
        self._publish_markers()

        self.get_logger().info(
            f'Previewing {len(self.waypoints)} waypoints for "{self.robot_name}" '
            f'on topic "waypoint_markers". Nav2 is not being used; nothing will move.'
        )

    def _resolve_robot_name(self) -> str:
        configured_name = str(self.get_parameter('robot_name').value).strip().strip('/')
        if configured_name:
            return configured_name
        namespace = self.get_namespace().strip('/')
        return namespace or 'parrot1'

    def _publish_markers(self) -> None:
        # current_index=-1: no mission running yet, so show the whole route
        # as upcoming rather than highlighting a "current" target.
        markers = mw.build_marker_array(self.waypoints, self.map_frame, -1, self.get_clock().now().to_msg())
        self.marker_pub.publish(markers)


def main(args: Optional[List[str]] = None):
    rclpy.init(args=args)
    node = WaypointPreview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
