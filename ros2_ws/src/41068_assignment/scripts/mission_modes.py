"""The flying modes other than Mapping: fly a path (Passive, Return Home) and hover (Active).

Not an executable node: mission_manager.py runs them. Each mode has the same
small interface - start(), step() -> status text, stop(), and a finished
flag - so a new mode (e.g. the next stage of Active mode) slots in the same
way.

Passive mode and Return Home fly above the trees, so they do not use Nav2:
at cruise altitude the lidar sees nothing (0 of 360 beams return at 7.8 m),
so there is nothing to avoid, and the tree footprints Nav2 learned at
mapping altitude would only push the drone off the trail. A pure-pursuit
follower steers along the path instead: aim at the point a fixed lookahead
distance ahead on the path, turn towards it, and slow down when the heading
is far off. The drone turns to face where it is going, so the forward,
downward-tilted cameras look along the trail ahead - where a hiker would be.
"""

import math
from dataclasses import dataclass
from typing import List, Tuple

from geometry_msgs.msg import Pose2D, Twist


@dataclass
class FollowConfig:
    lookahead: float = 2.0         # pure-pursuit lookahead distance (m)
    max_speed: float = 1.0         # forward speed limit (m/s), as Nav2's max_vel_x
    max_yaw_rate: float = 1.0      # turn rate limit (rad/s)
    yaw_gain: float = 1.5          # turn rate per radian of heading error (1/s)
    goal_tolerance: float = 0.5    # path end counts as reached within this (m)


def pure_pursuit(pose: Pose2D, path: List[Tuple[float, float]], progress: int,
                 config: FollowConfig) -> Tuple[float, float, int, bool]:
    """One pure-pursuit step: (forward speed, yaw rate, new progress, done).

    progress is the index of the path point the drone has got to; it only
    moves forward, so a path that doubles back near itself is still followed
    in order.
    """
    # Advance progress past every point already within the lookahead
    while (progress < len(path) - 1
           and math.hypot(path[progress][0] - pose.x, path[progress][1] - pose.y)
           < config.lookahead):
        progress += 1

    end_x, end_y = path[-1]
    to_end = math.hypot(end_x - pose.x, end_y - pose.y)
    if progress == len(path) - 1 and to_end < config.goal_tolerance:
        return 0.0, 0.0, progress, True

    target_x, target_y = path[progress]
    bearing = math.atan2(target_y - pose.y, target_x - pose.x)
    error = math.atan2(math.sin(bearing - pose.theta), math.cos(bearing - pose.theta))

    yaw_rate = max(-config.max_yaw_rate, min(config.max_yaw_rate, config.yaw_gain * error))
    # Full speed when facing the target, none when it is 90 deg or more off,
    # and slowing over the last metre so the end is not overshot.
    speed = config.max_speed * max(0.0, math.cos(error)) * min(1.0, to_end)
    return speed, yaw_rate, progress, False


class FlyPathMode:
    """Climb on the spot, follow a path at that altitude, optionally descend."""

    def __init__(self, manager, name: str, path: List[Tuple[float, float]],
                 cruise_altitude: float, final_altitude: float = None):
        self.manager = manager
        self.name = name
        self.path = path
        self.cruise_altitude = cruise_altitude
        self.final_altitude = final_altitude      # None: stay at cruise altitude
        self.config = manager.follow_config
        self.progress = 0
        self.phase = 'climb'
        self.finished = False

    def start(self):
        self.manager.set_altitude(self.cruise_altitude)

    def stop(self):
        self.manager.publish_cmd(Twist())

    def step(self) -> str:
        manager = self.manager
        if self.phase == 'climb':
            # Climb before moving, so the drone is above the trees before it
            # flies over them.
            if not manager.at_altitude(self.cruise_altitude):
                return f'climbing to {self.cruise_altitude:.1f} m'
            self.phase = 'follow'

        if self.phase == 'follow':
            pose = manager.get_pose()
            if pose is None:
                return 'waiting for the drone pose'
            speed, yaw_rate, self.progress, done = pure_pursuit(
                pose, self.path, self.progress, self.config)
            cmd = Twist()
            cmd.linear.x = speed
            cmd.angular.z = yaw_rate
            manager.publish_cmd(cmd)
            if not done:
                return f'following path, waypoint {self.progress + 1}/{len(self.path)}'
            manager.publish_cmd(Twist())
            if self.final_altitude is None:
                self.finished = True
                return 'reached the end of the path'
            manager.set_altitude(self.final_altitude)
            self.phase = 'descend'

        if self.phase == 'descend':
            if manager.at_altitude(self.final_altitude):
                self.finished = True
                return 'landed at the end of the path'
            return f'descending to {self.final_altitude:.1f} m'
        return self.phase


class HoverMode:
    """Hold position at an altitude. Used for Idle and as the first step of Active.

    Active mode's next steps (circling the person, confirming they need help,
    calling up a ranger route) are still to be designed; they belong here.
    """

    def __init__(self, manager, name: str, altitude: float = None):
        self.manager = manager
        self.name = name
        self.altitude = altitude          # None: keep whatever altitude is set
        self.finished = False

    def start(self):
        self.manager.publish_cmd(Twist())
        if self.altitude is not None:
            self.manager.set_altitude(self.altitude)

    def stop(self):
        pass

    def step(self) -> str:
        if self.altitude is not None and not self.manager.at_altitude(self.altitude):
            return f'moving to {self.altitude:.1f} m'
        return 'hovering'
