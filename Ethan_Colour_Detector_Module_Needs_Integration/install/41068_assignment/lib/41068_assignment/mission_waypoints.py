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
    -5.0, -3.0, 100.0,
]

# --- YOLO test route: fly up to the person and face them --------------------
# person1 ("Rescue Randy") is placed at (-5, 0) in assignment_world.sdf; keep
# PERSON_XY in sync if you move it there. The drone spawns at (0, -4).
PERSON_XY = (-5.0, 0.0)
DRONE_SPAWN_XY = (0.0, -4.0)

# Base-to-person distance (m) to stop at. The RGB-D camera is angled 45 deg
# down from ~1.05 m up with a ~98 deg vertical FOV, so a very close person is
# mostly above the frame. Around 4-5 m keeps most of the body in view while
# still being large enough (tens of pixels tall) for yolo26n to pick up.
PERSON_STANDOFF = 4.5


def approach_waypoints(person_xy, start_xy, standoffs):
    """Waypoints on the line from start_xy toward person_xy, each facing them.

    One (x, y, yaw_deg) triple per stand-off distance, flattened, in the order
    given (list far -> near to creep up on the person). Yaw points from the
    waypoint at the person, so the forward-facing camera looks at them.
    """
    px, py = person_xy
    dx, dy = start_xy[0] - px, start_xy[1] - py
    dist = math.hypot(dx, dy)
    ux, uy = dx / dist, dy / dist  # unit vector person -> start
    flat = []
    for d in standoffs:
        x, y = px + ux * d, py + uy * d
        yaw_deg = math.degrees(math.atan2(py - y, px - x))
        flat += [round(x, 2), round(y, 2), round(yaw_deg, 1)]
    return flat


# Far look first, then the stand-off point. Both face the person.
PERSON_APPROACH_WAYPOINTS = approach_waypoints(
    PERSON_XY, DRONE_SPAWN_XY, [PERSON_STANDOFF + 1.5, PERSON_STANDOFF],
)

# --- Line inspection route: look at the three people --------------------------
# assignment_world.sdf has three people in a line along y at x = -5, 2 m apart:
# person_red, person1 (normal) and person_green. Keep these in sync with the
# poses in the world file.
LINE_PEOPLE_XY = {
    'person_red': (-5.0, -2.0),
    'person1': (-5.0, 0.0),
    'person_green': (-5.0, 2.0),
}
LINE_CENTER_XY = (-5.0, 0.0)

# Hold point distance (m) from the middle of the line, straight out along x so
# the camera looks square-on at all three people. At 7 m the people sit within
# about +/-16 deg of the camera axis, well inside its 120 deg horizontal FOV.
# The camera is tilted 45 deg down, though, so the top of the frame is only a
# few degrees above horizontal: at 7 m it can clip the top of a standing
# person's head. Raise this (e.g. 9-10) to get heads fully in frame.
LINE_STANDOFF = 7.0
# The approach from the spawn point is split into this many short hops.
LINE_APPROACH_HOPS = 3
# Yaw step (deg) for the turn on the spot, and the pause (s) after each step.
LINE_TURN_STEP_DEG = 30.0
LINE_PAUSE_SECONDS = 2.5
# Nav2's shared config allows 1.57 rad of yaw error, which would swallow every
# small turn step, so this route tightens it (radians) at startup.
LINE_YAW_TOLERANCE = 0.10


def _wrap_deg(angle):
    """Wrap an angle in degrees to (-180, 180]."""
    wrapped = (angle + 180.0) % 360.0 - 180.0
    return 180.0 if wrapped == -180.0 else wrapped


def line_inspection_waypoints(start_xy, line_center_xy, standoff, hops, turn_step_deg):
    """Flat waypoint list: fly out facing away, then turn on the spot to the line.

    1. `hops` short hops from start_xy to a hold point `standoff` metres from
       the middle of the line (on the line's perpendicular bisector, on the
       start side), all facing away from the line.
    2. Turn in place, `turn_step_deg` at a time, until facing the line. The last
       waypoint is the hold point facing the line, and the route ends there.
    """
    cx, cy = line_center_xy
    sx, sy = start_xy
    # Line runs along y, so its perpendicular is the x axis: hold on whichever
    # side of the line the drone starts.
    side = 1.0 if sx >= cx else -1.0
    hold_x, hold_y = cx + side * standoff, cy

    face_deg = math.degrees(math.atan2(cy - hold_y, cx - hold_x))  # toward the line
    away_deg = _wrap_deg(face_deg + 180.0)

    flat = []
    for i in range(1, hops + 1):
        t = i / hops
        flat += [
            round(sx + (hold_x - sx) * t, 2),
            round(sy + (hold_y - sy) * t, 2),
            round(away_deg, 1),
        ]

    # Turn the short way round... counter-clockwise from facing away to facing
    # the line is exactly 180 deg, so the direction is a free choice.
    steps = max(1, math.ceil(180.0 / turn_step_deg))
    for k in range(1, steps + 1):
        yaw = _wrap_deg(away_deg + min(k * turn_step_deg, 180.0))
        flat += [round(hold_x, 2), round(hold_y, 2), round(yaw, 1)]
    return flat


LINE_INSPECTION_WAYPOINTS = line_inspection_waypoints(
    DRONE_SPAWN_XY, LINE_CENTER_XY, LINE_STANDOFF, LINE_APPROACH_HOPS, LINE_TURN_STEP_DEG,
)

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