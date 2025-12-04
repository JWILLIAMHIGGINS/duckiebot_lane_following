#!/usr/bin/env python3

# OMAR

import os
import math
import rospy
from duckietown.dtros import DTROS, NodeType
from geometry_msgs.msg import Point
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import Float64

class LaneControllerNode(DTROS):
    def __init__(self, node_name):
        super(LaneControllerNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)

        # Declare and get parameters
        # Set default parameters on parameter server
        if not rospy.has_param('~k1'):
            rospy.set_param('~k1', 1.0)
        if not rospy.has_param('~k2'):
            rospy.set_param('~k2', 1.0)
        if not rospy.has_param('~k3'):
            rospy.set_param('~k3', 1.0)
        if not rospy.has_param('~kp'):
            rospy.set_param('~kp', 2.0)
        if not rospy.has_param('~v'):
            rospy.set_param('~v', 0.01)  # forward speed (m/s)
        if not rospy.has_param('~L'):
            rospy.set_param('~L', 0.094)  # wheelbase (m)
        if not rospy.has_param('~R'):
            rospy.set_param('~R', 0.031)  # wheel radius (m)
        if not rospy.has_param('~omega_max'):
            rospy.set_param('~omega_max', 5.0)  # rad/s
        if not rospy.has_param('~wheel_speed_max'):
            rospy.set_param('~wheel_speed_max', 10.0)  # rad/s

        # Initialize feature values
        self.x_v = None
        self.x_m = None

        self._vehicle_name = os.environ.get('VEHICLE_NAME', 'deutschbot')

        # Subscribers
        self.vanish_sub = rospy.Subscriber(f"/{self._vehicle_name}/lane_following/vanishing_point", Point, self.vanishing_callback, queue_size=10)
        self.mid_sub = rospy.Subscriber(f"/{self._vehicle_name}/lane_following/mid_point", Point, self.middle_callback, queue_size=10)

        # Publisher
        self.wheels_pub = rospy.Publisher(f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=10)
        self.omega_pub = rospy.Publisher(f"/{self._vehicle_name}/lane_controller/omega", Float64, queue_size=10)
        
        # Register shutdown hook to stop wheels when node dies
        rospy.on_shutdown(self.shutdown_hook)
        
        self.log("LaneControllerNode initialized.")

    def vanishing_callback(self, msg):
        self.x_v = msg.x
        self.try_compute_control()

    def middle_callback(self, msg):
        self.x_m = msg.x
        self.try_compute_control()

    def try_compute_control(self):
        if self.x_v is None or self.x_m is None:
            return
        self.log(f"Computing control with x_v: {self.x_v}, x_m: {self.x_m}")
        # Read parameters here instead of init
        self.k1 = rospy.get_param('~k1', 1.0)
        self.k2 = rospy.get_param('~k2', 1.0)
        self.k3 = rospy.get_param('~k3', 1.0)
        self.kp = rospy.get_param('~kp', 2.0)
        self.v = rospy.get_param('~v', 0.2)  # forward speed (m/s)
        self.L = rospy.get_param('~L', 0.094)  # wheelbase (m)
        self.R = rospy.get_param('~R', 0.031)  # wheel radius (m)
        self.omega_max = rospy.get_param('~omega_max', 5.0)  # rad/s
        self.wheel_speed_max = rospy.get_param('~wheel_speed_max', 10.0)  # rad/s

        # Compute omega using control law (Eq. 1)
        denom = self.k1 * self.k3 + self.x_m * self.x_v
        if abs(denom) < 1e-6:
            self.log('Denominator too small, skipping control update.')
            return

        omega = (self.k1 / denom) * (-(self.k2 / self.k1) * self.v * self.x_v - self.kp * self.x_m)
        omega = max(-self.omega_max, min(self.omega_max, omega))

        # Publish omega control output
        omega_msg = Float64()
        omega_msg.data = omega
        self.omega_pub.publish(omega_msg)

        # Convert to wheel angular velocities (rad/s)
        # Differential drive equations: v = R*(w_r + w_l)/2, omega = R*(w_r - w_l)/L
        # Solving for w_l and w_r:
        v_l = (self.v - omega * self.L / 2) / self.R
        v_r = (self.v + omega * self.L / 2) / self.R

        v_l = max(-self.wheel_speed_max, min(self.wheel_speed_max, v_l))
        v_r = max(-self.wheel_speed_max, min(self.wheel_speed_max, v_r))

        # Publish
        cmd = WheelsCmdStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.vel_left = float(v_l)
        cmd.vel_right = float(v_r)
        self.wheels_pub.publish(cmd)

    def shutdown_hook(self):
        """Called when node is shutting down - stop the wheels"""
        self.log("Shutting down - stopping wheels")
        
        # Send zero velocity command
        cmd = WheelsCmdStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.vel_left = 0.0
        cmd.vel_right = 0.0
        self.wheels_pub.publish(cmd)
        
        # Publish zero omega
        omega_msg = Float64()
        omega_msg.data = 0.0
        self.omega_pub.publish(omega_msg)
        
        # Give it a moment to publish
        rospy.sleep(0.1)


def main():
    # create the node
    node = LaneControllerNode(node_name='lane_controller_node')
    # keep spinning
    rospy.spin()


if __name__ == '__main__':
    main()
