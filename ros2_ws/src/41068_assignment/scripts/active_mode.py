"""Active mode: fly to a found red-shirted person and confirm them close up.

Not an executable node: mission_manager.py runs it. The person comes from
target_projector's people list (points_of_interest), where YOLO, the depth
camera and the thermal camera have already agreed on them in several frames.
The approach itself is AT3's close-range inspection (person_approach.py):
pick a standoff on a ring round the person with a clear view, fly there with
Nav2, turn to face them, and count it as visited only if they are seen again.

Started by the `active` command it hovers until such a person appears.
Started by Passive mode, which found one on its trail, it already has them.
When the approach is visited or abandoned it finishes: mission_manager
switches to the mode it was given to resume (the rest of the trail) or goes
Idle. Either way the person's id is handled and does not start Active mode
again.

It never climbs. Passive flies at 0.8 m, under the oak canopy (from 1.04 m),
and a climb straight up to active_altitude from there would fly into the
branches. From cruise altitude it still drops to active_altitude first.
"""

import math

from geometry_msgs.msg import Twist

from coverage_explorer import MapGrid
from nav_goal import NavGoal
from person_approach import ApproachPlanner, Phase


class ActiveMode:
    name = 'ACTIVE'

    def __init__(self, manager, person: dict = None, resume=None):
        self.manager = manager            # mission_manager.MissionManager
        self.finished = False
        self.resume = resume              # mode to switch to when done (None: Idle)

        self.altitude = manager.get_parameter('active_altitude').value
        self.person = None                # points_of_interest entry: id, x, y, type, ...
        self.planner = None               # made once at altitude with a person
        self.nav = NavGoal(manager)
        self.planned_stamp = None         # map the last fruitless plan() ran on
        if person is not None:
            self.take(person)

    def start(self):
        self.manager.publish_cmd(Twist())
        if self.manager.altitude is not None:
            self.altitude = min(self.altitude, self.manager.altitude)
        self.manager.set_altitude(self.altitude)

    def stop(self):
        self.nav.cancel()
        self.manager.publish_cmd(Twist())

    def wants_person(self) -> bool:
        return self.person is None

    def take(self, person: dict):
        self.person = person
        self.manager.get_logger().info(
            f'Active: {person["id"]} confirmed (red, body heat) at '
            f'({person["x"]:.2f}, {person["y"]:.2f}), approaching')

    def step(self) -> str:
        manager = self.manager
        if not manager.at_altitude(self.altitude):
            return f'moving to {self.altitude:.1f} m'
        if self.person is None:
            return 'hovering, watching for a red-shirted person'

        now = manager.now()
        if self.planner is None:
            self.planner = ApproachPlanner(manager.approach_config, self.person, now)
        planner = self.planner
        planner.update_person(manager.people)

        result = self.nav.take_result()
        if result is not None:
            planner.on_goal_result(result, now)
        planner.tick(manager.people, now)

        if planner.phase == Phase.PLANNING:
            status = self.plan(now)
        elif planner.phase == Phase.APPROACHING:
            goal = planner.goal
            hop = ' (hop)' if planner.is_hop else ''
            status = f'flying to [{goal.x:.1f}, {goal.y:.1f}]{hop}'
        elif planner.phase == Phase.VERIFYING:
            self.face_person()
            status = 'at the standoff, looking for them'
        else:
            status = self.finish()

        # Stop a goal the planner gave up on (timeout, replan, abandon).
        if planner.phase != Phase.APPROACHING and self.nav.running:
            self.nav.cancel()
        p = self.person
        attempt = f', attempt {planner.attempts}' if planner.attempts > 1 else ''
        return f'{p["id"]} at ({p["x"]:.1f}, {p["y"]:.1f}): {status}{attempt}'

    def plan(self, now: float) -> str:
        """Send the next approach goal, if the map and pose allow one yet."""
        manager = self.manager
        pose = manager.get_pose()
        if manager.terrain_grid is None or pose is None:
            return 'waiting for the terrain map and pose'
        # No standoff on this map: wait for the next one rather than rerun
        # the wavefront every step.
        if manager.terrain_grid.header.stamp == self.planned_stamp:
            return 'looking for a standoff with a view of them'
        grid = MapGrid.from_msg(manager.terrain_grid)
        # Mapping's wavefront, from the drone, as AT3 used Planning 1's.
        manager.explorer.update_map(grid)
        row, col = grid.world_to_grid(pose.x, pose.y)
        dist_field = manager.explorer.compute_distance_field(row, col)
        goal = self.planner.plan(grid, dist_field, pose, now)
        if goal is None:
            self.planned_stamp = manager.terrain_grid.header.stamp
            return 'looking for a standoff with a view of them'
        self.planned_stamp = None
        self.nav.send(goal)
        return f'flying to [{goal.x:.1f}, {goal.y:.1f}]'

    def face_person(self):
        """Turn on the spot towards the person: Nav2 ends with any yaw (1.57 rad tolerance)."""
        pose = self.manager.get_pose()
        if pose is None:
            return
        px, py = self.planner.person_xy
        bearing = math.atan2(py - pose.y, px - pose.x)
        error = math.atan2(math.sin(bearing - pose.theta), math.cos(bearing - pose.theta))
        cfg = self.manager.follow_config
        cmd = Twist()
        cmd.angular.z = max(-cfg.max_yaw_rate, min(cfg.max_yaw_rate, cfg.yaw_gain * error))
        self.manager.publish_cmd(cmd)

    def finish(self) -> str:
        planner = self.planner
        self.finished = True
        self.manager.publish_cmd(Twist())
        p = self.person
        if planner.visited:
            self.manager.get_logger().info(
                f'Active: {p["id"]} visited from ({planner.standoff.x:.1f}, '
                f'{planner.standoff.y:.1f})')
            return 'visited'
        self.manager.get_logger().warn(f'Active: {p["id"]} abandoned: {planner.reason}')
        return f'abandoned: {planner.reason}'
