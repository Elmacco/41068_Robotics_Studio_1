"""Passive mode: patrol one detected trail low, watching for people.

Not an executable node: mission_manager.py runs it. It needs trails, so it
runs after Mapping (or from a loaded map): the `passive` command names a
trail, or takes the one nearest the drone.

The drone flies the trail at mapping altitude (0.8 m), not above the trees:

- target_projector places a person from the depth camera, which reaches
  10 m. From 8 m, with the camera tilted 45 deg down, that is only people
  within about 6 m of the drone; at 0.8 m it is the trail ahead, as Mapping
  showed.
- Active mode can then fly to a person Passive found without changing
  altitude. simple_trees' trail runs past the oaks, whose canopy starts at
  1.04 m, so a descent from above the trees could land in one.

Down there are trunks, and people on the trail, so Nav2 does the flying, as
in Mapping. The trail is cut into waypoints every `spacing` m and Nav2 is sent
to one at a time, facing along the trail, so the forward camera looks where a
hiker would be. Within advance_radius of a waypoint the next one is sent (Nav2
preempts the running goal), so the drone keeps moving instead of stopping at
each one. A waypoint Nav2 cannot reach (a person standing on it, say) is
skipped; max_skips in a row ends the patrol. A first waypoint beyond Nav2's
rolling costmap is reached in hops (hop_towards), as Active mode does.

A red-shirted person seen on the way starts Active mode (mission_manager's
check_people), which then resumes with remaining(): the rest of the trail
from the waypoint the drone was heading to.
"""

import math
from dataclasses import dataclass
from typing import List, Tuple

from geometry_msgs.msg import Pose2D, Twist

from coverage_explorer import MapGrid
from nav_goal import NavGoal
from person_approach import clearance_map, hop_towards


@dataclass
class PassiveConfig:
    spacing: float = 2.0           # trail waypoints every this far (m)
    advance_radius: float = 1.0    # send the next waypoint within this of the current one (m)
    max_skips: int = 3             # unreachable waypoints in a row before giving up


class PassiveMode:

    def __init__(self, manager, trail: str, waypoints: List[Tuple[float, float]],
                 altitude: float):
        self.manager = manager            # mission_manager.MissionManager
        self.trail = trail
        self.name = f'PASSIVE {trail}'
        self.waypoints = list(waypoints)
        self.altitude = altitude
        self.config = manager.passive_config
        self.nav = NavGoal(manager)
        self.index = 0                    # waypoint the drone is heading to
        self.sent_index = None            # waypoint the running goal is for
        self.is_hop = False               # running goal is a hop towards it
        self.skips = 0                    # unreachable waypoints in a row
        self.finished = False

    def start(self):
        self.manager.publish_cmd(Twist())
        self.manager.set_altitude(self.altitude)

    def stop(self):
        self.nav.cancel()
        self.manager.publish_cmd(Twist())

    def remaining(self) -> 'PassiveMode':
        """The rest of the trail, from the waypoint the drone was heading to."""
        return PassiveMode(self.manager, self.trail, self.waypoints[self.index:], self.altitude)

    def step(self) -> str:
        manager = self.manager
        if not manager.at_altitude(self.altitude):
            return f'moving to {self.altitude:.1f} m'
        pose = manager.get_pose()
        if pose is None:
            return 'waiting for the drone pose'

        result = self.nav.take_result()
        if result is not None and not self.is_hop:
            # Reached (Nav2's tolerance may be wider than advance_radius) or
            # unreachable: either way move on to the next waypoint.
            self.index += 1
        if result is False:
            self.skips += 1
            if self.skips > self.config.max_skips:
                return self.finish(f'gave up: {self.skips} waypoints in a row unreachable')
        elif result is True and not self.is_hop:
            self.skips = 0

        while (self.index < len(self.waypoints)
               and self.distance_to(pose, self.index) < self.config.advance_radius):
            self.index += 1
            self.skips = 0
        if self.index >= len(self.waypoints):
            return self.finish('reached the end of the trail')

        if not self.nav.running or self.sent_index != self.index:
            self.send_goal(pose)
        hop = ' (hop)' if self.is_hop else ''
        return f'to waypoint {self.index + 1}/{len(self.waypoints)}{hop}'

    def send_goal(self, pose: Pose2D):
        x, y = self.waypoints[self.index]
        goal = Pose2D(x=float(x), y=float(y), theta=self.heading_at(self.index, pose))
        far = math.hypot(x - pose.x, y - pose.y) > self.manager.approach_config.max_goal_distance
        self.is_hop = far and self.manager.terrain_grid is not None
        if self.is_hop:
            grid = MapGrid.from_msg(self.manager.terrain_grid)
            self.manager.explorer.update_map(grid)
            dist_field = self.manager.explorer.compute_distance_field(*grid.world_to_grid(pose.x, pose.y))
            goal = hop_towards(goal, pose, grid, dist_field, clearance_map(grid),
                               self.manager.approach_config)
        self.nav.send(goal)
        self.sent_index = self.index

    def heading_at(self, i: int, pose: Pose2D) -> float:
        """Along the trail at waypoint i: towards the next one (or from the last)."""
        x, y = self.waypoints[i]
        if i + 1 < len(self.waypoints):
            next_x, next_y = self.waypoints[i + 1]
            return math.atan2(next_y - y, next_x - x)
        prev_x, prev_y = self.waypoints[i - 1] if i > 0 else (pose.x, pose.y)
        return math.atan2(y - prev_y, x - prev_x)

    def distance_to(self, pose: Pose2D, i: int) -> float:
        x, y = self.waypoints[i]
        return math.hypot(x - pose.x, y - pose.y)

    def finish(self, reason: str) -> str:
        self.nav.cancel()
        self.manager.publish_cmd(Twist())
        self.finished = True
        return reason
