"""Mapping mode: explore the world with Nav2 until the camera has seen all of it.

Not an executable node: mission_manager.py runs it.

The goal handling is AT3's (cave_explorer.py), unchanged in substance:
  - each goal gets a sequence number, so a late result from a goal that was
    already given up on cannot tear down the one that replaced it
  - Nav2 reports aborts through the same callback as successes, so the
    status is checked; a failed goal is blacklisted so it is not chosen again
  - a goal reported reached within a second of being sent was already
    inside Nav2's goal tolerance and revealed nothing, so it is blacklisted
    too (AT3 saw one goal re-sent 241 times without this)
  - a goal whose frontier the camera has already seen on the way is
    abandoned early, without blacklisting
  - a goal that takes too long is cancelled and blacklisted
  - exploration is only declared complete after several consecutive
    selections find nothing, since one empty result can be a transient map

New here: the drone first settles at mapping_altitude. Low enough that the
lidar plane (0.2 m above the drone) sees tree trunks, which is also roughly
the height of what a walking person would hit, and the camera sees the
ground under the canopy.

Also new: goals are judged by the information they bring (GoalWatchdog),
not only by time. A goal is given up once, for stall_time, the camera has
seen no new ground and the drone has got no closer; and a goal that is
reached having seen almost nothing new is blacklisted like a failed one.
Both are blacklisted over a wider radius than a plain failure. Without this
the drone circled the person by the launch pad in simple_trees: the ground
behind a standing person is hidden from a camera 1 m up, so a frontier
stays there, the nearest one, and reaching it (facing the person) revealed
nothing, so it was chosen again and again. The fixed timeout is now only a
backstop.

Also new: a frontier further than Nav2's rolling costmap reaches is flown to
in hops (CoverageExplorer.select_goal). A hop is a stopover, not a place to
look from: reaching one having seen nothing new is still a success, and it
is only stale once its frontier, not the hop itself, has been seen.
"""

import math
from typing import Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose2D, PoseStamped
from nav2_msgs.action import NavigateToPose

from coverage_explorer import CoverageExplorer, MapGrid


class GoalWatchdog:
    """Decides when a Mapping goal has stopped paying off. No ROS, so it can be tested.

    Two kinds of progress count: new ground seen by the camera (the point of
    Mapping mode) and getting closer to the goal (on the way to new ground,
    crossing ground already mapped). A goal with neither for stall_time has
    stalled: stuck, or at a frontier it cannot see into.
    """

    def __init__(self, stall_time: float, min_gain_area: float, min_progress: float,
                 timeout: float):
        self.stall_time = stall_time          # s with no progress of either kind
        self.min_gain_area = min_gain_area    # m2 of new ground that counts as progress
        self.min_progress = min_progress      # m closer to the goal that counts
        self.timeout = timeout                # s, backstop whatever happens
        self.start_time = self.start_area = 0.0
        self.last_useful = self.last_area = 0.0
        self.best_distance = math.inf

    def start(self, now: float, observed_area: float, distance: float):
        self.start_time = self.last_useful = now
        self.start_area = self.last_area = observed_area
        self.best_distance = distance

    def gain(self, observed_area: float) -> float:
        """New ground seen since the goal was sent (m2)."""
        return observed_area - self.start_area

    def update(self, now: float, observed_area: float, distance: float) -> Optional[str]:
        """Why the goal should be given up, or None to keep going."""
        if observed_area - self.last_area >= self.min_gain_area:
            self.last_area = observed_area
            self.last_useful = now
        if distance <= self.best_distance - self.min_progress:
            self.best_distance = distance
            self.last_useful = now
        if now - self.start_time >= self.timeout:
            return f'timed out after {now - self.start_time:.0f} s'
        if now - self.last_useful >= self.stall_time:
            return (f'no new ground and no closer for {now - self.last_useful:.0f} s '
                    f'({self.gain(observed_area):.1f} m2 seen)')
        return None


class MappingMode:
    name = 'MAPPING'

    def __init__(self, manager, explorer: CoverageExplorer):
        self.manager = manager            # mission_manager.MissionManager
        self.explorer = explorer
        self.finished = False

        params = manager.mapping_params
        self.altitude = params['mapping_altitude']
        self.no_goal_limit = params['no_goal_limit']
        self.min_goal_gain = params['min_goal_gain']
        self.no_gain_blacklist_radius = params['no_gain_blacklist_radius']
        self.watchdog = GoalWatchdog(params['stall_time'], params['min_gain_area'],
                                     params['min_progress'], params['goal_timeout'])

        self.no_goal_count = 0
        self.grid_stamp = None            # stamp of the map frontiers were last found on
        self.goal_seq = 0                 # increments per goal sent
        self.active_seq: Optional[int] = None
        self.goal_handle = None
        self.goal_start = None
        self.waiting_for_result = False
        self.status = 'starting'

    # ---------------------------------------------------------------- lifecycle

    def start(self):
        self.manager.set_altitude(self.altitude)
        self.explorer.clear_selection()

    def stop(self):
        self.cancel_goal()
        self.explorer.clear_selection()

    def step(self) -> str:
        manager = self.manager
        if not manager.at_altitude(self.altitude):
            return f'climbing to {self.altitude:.1f} m'

        # Re-detect frontiers whenever terrain_mapper publishes a new map
        grid_msg = manager.terrain_grid
        if grid_msg is None:
            return 'waiting for the terrain map'
        if grid_msg.header.stamp != self.grid_stamp:
            self.grid_stamp = grid_msg.header.stamp
            self.explorer.update_map(MapGrid.from_msg(grid_msg))
            self.explorer.find_frontiers()
            manager.publish_exploration_markers(self.explorer)
            self.abandon_stale_goal()

        coverage = 100 * self.explorer.exploration_stats()['coverage']

        if self.waiting_for_result:
            self.check_goal_progress()
            return f'{coverage:.0f}% observed, {self.status}'

        pose = manager.get_pose()
        if pose is None:
            return 'waiting for the drone pose'

        goal = self.explorer.select_goal(pose)
        if goal is None:
            # Several empty results in a row before stopping: one can happen
            # while the map is momentarily inconsistent (AT3).
            self.no_goal_count += 1
            if self.no_goal_count >= self.no_goal_limit:
                self.finished = True
                return f'complete: {coverage:.0f}% observed, no reachable frontier left'
            return f'{coverage:.0f}% observed, no reachable frontier ' \
                   f'({self.no_goal_count}/{self.no_goal_limit})'

        self.no_goal_count = 0
        frontier = self.explorer.selected_
        self.status = f'flying to F{frontier.frontier_id} at [{goal.x:.1f}, {goal.y:.1f}]'
        if self.explorer.is_waypoint_:
            target = self.explorer.target_pose_
            self.status = (f'flying to F{frontier.frontier_id} at [{target.x:.1f}, {target.y:.1f}]'
                           f' via [{goal.x:.1f}, {goal.y:.1f}]')
        self.send_goal(goal)
        return f'{coverage:.0f}% observed, {self.status}'

    # -------------------------------------------------------- Nav2 goals (AT3)

    def send_goal(self, pose2d: Pose2D):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.manager.frame_id
        goal.pose.header.stamp = self.manager.get_clock().now().to_msg()
        goal.pose.pose.position.x = pose2d.x
        goal.pose.pose.position.y = pose2d.y
        goal.pose.pose.orientation.z = math.sin(pose2d.theta / 2.0)
        goal.pose.pose.orientation.w = math.cos(pose2d.theta / 2.0)

        self.goal_seq += 1
        seq = self.goal_seq
        self.active_seq = seq
        self.goal_start = self.manager.get_clock().now()
        self.watchdog.start(self.now(), self.observed_area(), self.distance_to(pose2d))
        self.waiting_for_result = True
        future = self.manager.nav_client.send_goal_async(goal)
        future.add_done_callback(lambda f, seq=seq: self.goal_response(f, seq))

    def goal_response(self, future, seq):
        handle = future.result()
        if seq != self.active_seq:
            # Superseded while the request was in flight; stop it if it started
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.manager.get_logger().warn('Nav2 rejected the goal')
            self.goal_finished(success=False)
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f, seq=seq: self.goal_result(f, seq))

    def goal_result(self, future, seq):
        if seq != self.active_seq:
            return                         # a goal already given up on
        success = future.result().status == GoalStatus.STATUS_SUCCEEDED
        if not success:
            self.manager.get_logger().warn(
                f'Goal did not succeed (status {future.result().status})')
        self.goal_finished(success)

    def goal_finished(self, success: bool, radius: Optional[float] = None):
        """Clear the goal; blacklist its frontier if it failed or brought nothing.

        Reached with less than min_goal_gain of new ground seen on the way:
        nothing to see there from where Nav2 stops (this also covers AT3's
        goal reached instantly, which sees nothing new). Blacklisted over
        no_gain_blacklist_radius, so the next goal is not the frontier cell
        beside it, with the same view blocked. A hop towards a far frontier
        is exempt: it is there to get within range, not to see anything.
        """
        gain = self.watchdog.gain(self.observed_area())
        if success and gain < self.min_goal_gain and not self.explorer.is_waypoint_:
            success = False
            radius = self.no_gain_blacklist_radius
            self.manager.get_logger().info(
                f'Goal reached but only {gain:.1f} m2 of new ground seen')
        if not success and self.explorer.goal_cell_xy_ is not None:
            x, y = self.explorer.goal_cell_xy_
            self.explorer.blacklist(x, y, radius)
            self.manager.get_logger().warn(
                f'Blacklisted goal [{x:.1f}, {y:.1f}] ({len(self.explorer.blacklist_)} total)')
        self.goal_handle = None
        self.active_seq = None
        self.waiting_for_result = False

    def abandon_stale_goal(self):
        """Give up on a goal whose frontier the camera has already seen (AT3)."""
        if not self.waiting_for_result:
            return
        goal = self.explorer.target_pose_       # the frontier end, not a hop
        if goal is None or self.explorer.goal_still_useful(goal):
            return
        self.manager.get_logger().info(
            f'Frontier at [{goal.x:.1f}, {goal.y:.1f}] already seen, picking a new goal')
        self.cancel_goal()

    def check_goal_progress(self):
        """Give up on a goal that has stopped paying off (GoalWatchdog)."""
        goal = self.explorer.goal_pose_
        if self.goal_start is None or goal is None:
            return
        reason = self.watchdog.update(self.now(), self.observed_area(), self.distance_to(goal))
        if reason is None:
            return
        self.manager.get_logger().warn(f'Goal {reason}, abandoning')
        self.cancel_goal()
        self.goal_finished(success=False, radius=self.no_gain_blacklist_radius)

    def now(self) -> float:
        return self.manager.get_clock().now().nanoseconds * 1e-9

    def observed_area(self) -> float:
        return self.explorer.exploration_stats()['observed_area_m2']

    def distance_to(self, goal: Pose2D) -> float:
        pose = self.manager.get_pose()
        if pose is None:
            return math.inf
        return math.hypot(goal.x - pose.x, goal.y - pose.y)

    def cancel_goal(self):
        """Stop the running goal without counting it as a failure."""
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal_handle = None
        self.active_seq = None
        self.waiting_for_result = False
