#!/usr/bin/env python3
"""Hold the drone's altitude while passing horizontal velocity commands through.

Gazebo moves the drone with whatever Twist last reached its cmd_vel topic, and
holds that velocity until the next one arrives. Nav2 only ever commands forward
speed and yaw rate (linear.z is always 0), so a separate altitude controller
publishing on the same topic would alternate with Nav2 and the two would fight.

This node is therefore the only publisher to Gazebo (cmd_vel_gz, which the
launch file remaps the Gazebo bridge onto). It takes the horizontal part of
whatever arrives on cmd_vel (Nav2, the dashboard's manual control, the passive
mode trail follower) and adds its own vertical velocity to hold target_altitude.
Any linear.z in the incoming command is ignored: altitude is owned here.

Altitude comes from Gazebo's 3D odometry topic (odometry), not the EKF or TF:
TF is kept 2D to match SLAM Toolbox and Nav2 (see spawn_parrot.launch.py).

Gazebo sets the velocity directly, so the drone's height is the integral of the
commanded vertical speed. For a plant like that, a proportional controller has
no steady-state error and needs no integral or derivative term: the height
converges exponentially with time constant 1 / kp.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64


class FlightController(Node):

    def __init__(self):
        super().__init__('flight_controller')

        # Initial altitude to hold, in metres (world z). The launch file sets
        # this to the spawn height so the drone holds where it appeared.
        self.declare_parameter('target_altitude', 0.8)
        # Vertical speed per metre of altitude error (1/s).
        self.declare_parameter('kp', 1.0)
        # Climb/descent speed limit (m/s).
        self.declare_parameter('max_vertical_speed', 1.0)
        # Horizontal commands older than this are treated as zero, so the drone
        # stops if whatever was steering it dies. The dashboard resends manual
        # commands every 0.3 s and Nav2 publishes at 20 Hz, so 1 s never trips
        # during normal use.
        self.declare_parameter('cmd_timeout', 1.0)
        self.declare_parameter('rate', 20.0)

        self.target_altitude = float(self.get_parameter('target_altitude').value)
        self.kp = float(self.get_parameter('kp').value)
        self.max_vertical_speed = float(self.get_parameter('max_vertical_speed').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        self.altitude = None              # latest measured altitude (m)
        self.last_cmd = Twist()           # latest horizontal command
        self.last_cmd_time = None         # when it arrived

        self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.create_subscription(Odometry, 'odometry', self.odom_callback, 10)
        self.create_subscription(Float64, 'target_altitude', self.target_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_gz', 10)

        rate = float(self.get_parameter('rate').value)
        self.create_timer(1.0 / rate, self.control_step)

        self.get_logger().info(f'Holding altitude {self.target_altitude:.2f} m')

    def cmd_callback(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def odom_callback(self, msg: Odometry):
        self.altitude = msg.pose.pose.position.z

    def target_callback(self, msg: Float64):
        if not math.isfinite(msg.data):
            self.get_logger().warn(f'Ignoring non-finite target altitude {msg.data}')
            return
        if abs(msg.data - self.target_altitude) > 1e-3:
            self.get_logger().info(f'New target altitude {msg.data:.2f} m')
        self.target_altitude = msg.data

    def control_step(self):
        # Without a height measurement there is nothing to control against.
        # Publishing nothing leaves Gazebo holding its last command, which at
        # startup is zero, so the drone simply hovers until odometry arrives.
        if self.altitude is None:
            return

        out = Twist()

        fresh = (self.last_cmd_time is not None
                 and (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
                 < self.cmd_timeout)
        if fresh:
            out.linear.x = self.last_cmd.linear.x
            out.linear.y = self.last_cmd.linear.y
            out.angular.z = self.last_cmd.angular.z

        error = self.target_altitude - self.altitude
        out.linear.z = max(-self.max_vertical_speed,
                           min(self.max_vertical_speed, self.kp * error))

        self.cmd_pub.publish(out)


def main():
    rclpy.init()
    node = FlightController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
