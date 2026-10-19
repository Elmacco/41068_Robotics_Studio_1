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
"""

import math
from typing import Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose2D, PoseStamped
from nav2_msgs.action import NavigateToPose

from coverage_explorer import CoverageExplorer, MapGrid


class MappingMode:
    name = 'MAPPING'

    def __init__(self, manager, explorer: CoverageExplorer):
        self.manager = manager            # mission_manager.MissionManager
        self.explorer = explorer
        self.finished = False

        params = manager.mapping_params
        self.altitude = params['mapping_altitude']
        self.goal_timeout = params['goal_timeout']
        self.no_goal_limit = params['no_goal_limit']

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
            manager.publish_exploration_markers(self.explorer.create_marker_array())
            self.abandon_stale_goal()

        coverage = 100 * self.explorer.exploration_stats()['coverage']

        if self.waiting_for_result:
            self.check_goal_timeout()
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

    def goal_finished(self, success: bool):
        elapsed = (self.manager.get_clock().now() - self.goal_start).nanoseconds * 1e-9
        instant = success and elapsed < 1.0
        if (not success or instant) and self.explorer.goal_cell_xy_ is not None:
            x, y = self.explorer.goal_cell_xy_
            self.explorer.blacklist(x, y)
            self.manager.get_logger().warn(
                f'Blacklisted goal [{x:.1f}, {y:.1f}] ({len(self.explorer.blacklist_)} total)')
        self.goal_handle = None
        self.active_seq = None
        self.waiting_for_result = False

    def abandon_stale_goal(self):
        """Give up on a goal whose frontier the camera has already seen (AT3)."""
        if not self.waiting_for_result:
            return
        goal = self.explorer.goal_pose_
        if goal is None or self.explorer.goal_still_useful(goal):
            return
        self.manager.get_logger().info(
            f'Frontier at [{goal.x:.1f}, {goal.y:.1f}] already seen, picking a new goal')
        self.cancel_goal()

    def check_goal_timeout(self):
        if self.goal_start is None:
            return
        elapsed = (self.manager.get_clock().now() - self.goal_start).nanoseconds * 1e-9
        if elapsed < self.goal_timeout:
            return
        self.manager.get_logger().warn(f'Goal timed out after {elapsed:.0f} s, abandoning')
        self.cancel_goal()
        self.goal_finished(success=False)

    def cancel_goal(self):
        """Stop the running goal without counting it as a failure."""
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal_handle = None
        self.active_seq = None
        self.waiting_for_result = False
