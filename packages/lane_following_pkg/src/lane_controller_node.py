#!/usr/bin/env python3

# OMAR

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from duckietown_msgs.msg import WheelsCmdStamped

class LaneControllerNode(Node):
    def __init__(self):
        super().__init__('lane_controller_node')

        # Declare and get parameters
        self.declare_parameter('k1', 1.0)
        self.declare_parameter('k2', 1.0)
        self.declare_parameter('k3', 1.0)
        self.declare_parameter('kp', 2.0)
        self.declare_parameter('v', 0.2)  # forward speed (m/s)
        self.declare_parameter('L', 0.094)  # wheelbase (m)
        self.declare_parameter('R', 0.031)  # wheel radius (m)
        self.declare_parameter('omega_max', 5.0)  # rad/s
        self.declare_parameter('wheel_speed_max', 10.0)  # rad/s

        self.k1 = self.get_parameter('k1').get_parameter_value().double_value
        self.k2 = self.get_parameter('k2').get_parameter_value().double_value
        self.k3 = self.get_parameter('k3').get_parameter_value().double_value
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.v = self.get_parameter('v').get_parameter_value().double_value
        self.L = self.get_parameter('L').get_parameter_value().double_value
        self.R = self.get_parameter('R').get_parameter_value().double_value
        self.omega_max = self.get_parameter('omega_max').get_parameter_value().double_value
        self.wheel_speed_max = self.get_parameter('wheel_speed_max').get_parameter_value().double_value

        # Initialize feature values
        self.x_v = None
        self.x_m = None

        # Subscribers
        self.create_subscription(Point, '/vanishing_point', self.vanishing_callback, 10)
        self.create_subscription(Point, '/mid_point', self.middle_callback, 10)

        # Publisher
        self.wheels_pub = self.create_publisher(WheelsCmdStamped, '/wheels_driver_node/wheels_cmd', 10)
        self.get_logger().info('LaneControllerNode initialized.')

    def vanishing_callback(self, msg):
        self.x_v = msg.x
        self.try_compute_control()

    def middle_callback(self, msg):
        self.x_m = msg.x
        self.try_compute_control()

    def try_compute_control(self):
        if self.x_v is None or self.x_m is None:
            return

        # Compute omega using control law (Eq. 1)
        denom = self.k1 * self.k3 + self.x_m * self.x_v
        if abs(denom) < 1e-6:
            self.get_logger().warn('Denominator too small, skipping control update.')
            return

        omega = (self.k1 / denom) * (-(self.k2 / self.k1) * self.v * self.x_v - self.kp * self.x_m)
        omega = max(-self.omega_max, min(self.omega_max, omega))

        # Convert to wheel angular velocities (rad/s)
        v_l = (2 * self.v - omega * self.L) / (2 * self.R)
        v_r = (2 * self.v + omega * self.L) / (2 * self.R)

        v_l = max(-self.wheel_speed_max, min(self.wheel_speed_max, v_l))
        v_r = max(-self.wheel_speed_max, min(self.wheel_speed_max, v_r))

        # Publish
        cmd = WheelsCmdStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.vel_left = float(v_l)
        cmd.vel_right = float(v_r)
        self.wheels_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LaneControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
