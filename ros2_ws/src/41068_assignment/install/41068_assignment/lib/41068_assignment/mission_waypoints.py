"""Shared waypoint route and RViz marker builder.

Not an executable node itself. waypoint_follower.py (drives the mission) and
publish_waypoints.py (preview only, no Nav2 calls) both import this so the
route and marker appearance stay in one place.
"""

import math

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

# (x, y, yaw_degrees) in the robot's map frame. Starts behind the single tree
# at (0, 3) in assignment_world.sdf (the drone spawns at (0, -4)) and ends
# beside it, clear of the canopy, so this exercises the movement/sensor
# pipeline without also depending on obstacle avoidance.
DEFAULT_WAYPOINTS = [
    -5.0, -3.0, 90.0,
    0.0, 1.0, 90.0,
    4.5, 3.0, 0.0,
]

# Matches the drone's flight altitude (the -z argument in spawn_parrot.launch.py).
MARKER_HEIGHT = 0.8


def parse_waypoints(flat):
    """Convert a flat [x, y, yaw_deg, x, y, yaw_deg, ...] list into triples."""
    flat = list(flat)
    if len(flat) % 3 != 0:
        raise ValueError('waypoints parameter must be a flat list of (x, y, yaw_degrees) triples')
    return [
        (flat[i], flat[i + 1], math.radians(flat[i + 2]))
        for i in range(0, len(flat), 3)
    ]


def build_marker_array(waypoints, map_frame, current_index, stamp):
    """Build a sphere+label MarkerArray for the route.

    current_index selects which waypoint is highlighted green as the active
    target; waypoints before it are grey (visited), after it are orange
    (upcoming). Pass -1 (e.g. before a mission starts) to show the whole
    route as upcoming.
    """
    markers = MarkerArray()
    for i, (x, y, _yaw) in enumerate(waypoints):
        if i < current_index:
            color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.6)
        elif i == current_index:
            color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        else:
            color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.8)

        sphere = Marker()
        sphere.header.frame_id = map_frame
        sphere.header.stamp = stamp
        sphere.ns = 'waypoints'
        sphere.id = i
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position = Point(x=x, y=y, z=MARKER_HEIGHT)
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.4
        sphere.color = color
        markers.markers.append(sphere)

        label = Marker()
        label.header.frame_id = map_frame
        label.header.stamp = stamp
        label.ns = 'waypoint_labels'
        label.id = i
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position = Point(x=x, y=y, z=MARKER_HEIGHT + 0.5)
        label.pose.orientation.w = 1.0
        label.scale.z = 0.3
        label.color = color
        label.text = str(i + 1)
        markers.markers.append(label)

    return markers
