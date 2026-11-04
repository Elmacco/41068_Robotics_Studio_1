"""One Nav2 NavigateToPose goal at a time, for Active and Passive mode.

Not an executable node: the modes own one each and poll it from step().
"""

import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose2D, PoseStamped
from nav2_msgs.action import NavigateToPose


class NavGoal:
    """One Nav2 NavigateToPose goal at a time, its result polled by step().

    MappingMode's bookkeeping (AT3): every goal gets a sequence number and
    replies about a superseded goal are ignored, so a late result from a
    replaced goal cannot end the one that replaced it.
    """

    def __init__(self, manager):
        self.manager = manager
        self.seq = 0
        self.active_seq = None
        self.handle = None
        self.result = None                # True / False once Nav2 finishes

    @property
    def running(self) -> bool:
        return self.active_seq is not None

    def send(self, pose2d: Pose2D):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.manager.frame_id
        goal.pose.header.stamp = self.manager.get_clock().now().to_msg()
        goal.pose.pose.position.x = pose2d.x
        goal.pose.pose.position.y = pose2d.y
        goal.pose.pose.orientation.z = math.sin(pose2d.theta / 2.0)
        goal.pose.pose.orientation.w = math.cos(pose2d.theta / 2.0)

        self.seq += 1
        seq = self.active_seq = self.seq
        self.result = None
        future = self.manager.nav_client.send_goal_async(goal)
        future.add_done_callback(lambda f, seq=seq: self._response(f, seq))

    def take_result(self):
        """Nav2's result for the running goal (once), or None if still running."""
        result, self.result = self.result, None
        return result

    def cancel(self):
        if self.handle is not None:
            self.handle.cancel_goal_async()
        self.handle = None
        self.active_seq = None
        self.result = None

    def _response(self, future, seq):
        handle = future.result()
        if seq != self.active_seq:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.manager.get_logger().warn('Nav2 rejected the goal')
            self._done(False)
            return
        self.handle = handle
        handle.get_result_async().add_done_callback(lambda f, seq=seq: self._result(f, seq))

    def _result(self, future, seq):
        if seq == self.active_seq:
            self._done(future.result().status == GoalStatus.STATUS_SUCCEEDED)

    def _done(self, success: bool):
        self.handle = None
        self.active_seq = None
        self.result = success
