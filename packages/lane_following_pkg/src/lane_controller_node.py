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
        if rospy.has_param('~k1'):
            rospy.set_param('~k1', 0.3)  # Reduced from 1.0
        if rospy.has_param('~k2'):
            rospy.set_param('~k2', 0.3)  # Reduced from 1.0
        if rospy.has_param('~k3'):
            rospy.set_param('~k3', 0.3)  # Reduced from 1.0
        if rospy.has_param('~kp'):
            rospy.set_param('~kp', 0.001)  # Reduced from 2.0 - this is the proportional gain for midpoint error
        if rospy.has_param('~ki'):
            rospy.set_param('~ki', 0.0001)  # Integral gain
        if rospy.has_param('~kd'):
            rospy.set_param('~kd', 0.00001)  # Derivative gain
        if not rospy.has_param('~v'):
            rospy.set_param('~v', 0.01)  # forward speed (m/s)
        if not rospy.has_param('~L'):
            rospy.set_param('~L', 0.094)  # wheelbase (m)
        if not rospy.has_param('~R'):
            rospy.set_param('~R', 0.031)  # wheel radius (m)
        if not rospy.has_param('~omega_max'):
            rospy.set_param('~omega_max', 2.0)  # Reduced from 5.0 - limit maximum turning rate
        if not rospy.has_param('~wheel_speed_max'):
            rospy.set_param('~wheel_speed_max', 5.0)  # Reduced from 10.0

        self.update_parameters()

        # Initialize feature values
        self.x_v = None
        self.x_m = None
        
        # PID controller state
        self.integral_error = 0.0
        self.last_error = 0.0
        self.last_time = None
        
        # Cache parameters (read once at init, update periodically)
        self.update_parameters()
        self.param_update_counter = 0

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
        self.try_compute_control()  # AKTIVIERT - jetzt läuft Controller bei jedem Topic-Update
    
    def update_parameters(self):
        """Update controller parameters from parameter server"""
        self.k1 = rospy.get_param('~k1', 0.5)
        self.k2 = rospy.get_param('~k2', 0.5)
        self.k3 = rospy.get_param('~k3', 0.5)
        self.L = rospy.get_param('~L', 0.094)
        self.R = rospy.get_param('~R', 0.031)
        self.omega_max = rospy.get_param('~omega_max', 2.0)
        self.wheel_speed_max = rospy.get_param('~wheel_speed_max', 5.0)
        self.update_ctrl_parameters()

    def update_ctrl_parameters(self):
        self.kp = rospy.get_param('~kp', 0.5)
        self.ki = rospy.get_param('~ki', 0.01)
        self.kd = rospy.get_param('~kd', 0.05)
        self.v = rospy.get_param('~v', 0.0)

    def try_compute_control(self):
        if self.x_v is None or self.x_m is None:
            return
        
        # Update parameters every 100 calls (~3 seconds at 30Hz) instead of every call
        self.param_update_counter += 1
        if self.param_update_counter >= 100:
            self.update_ctrl_parameters()
            self.param_update_counter = 0

        # Compute omega using control law (Eq. 1)
        denom = self.k1 * self.k3 + self.x_m * self.x_v
        if abs(denom) < 1e-6:
            self.log('Denominator too small, skipping control update.')
            return
        
        # PID Controller on vanishing point error
        # Error: we want vanishing point at center (x_v = 0)
        error = 0 - self.x_v
        
        # Calculate dt for integral and derivative terms
        current_time = rospy.Time.now()
        if self.last_time is None:
            dt = 0.0
        else:
            dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time
        
        # Integral term with anti-windup
        if dt > 0:
            self.integral_error += error * dt
            # Anti-windup: limit integral term
            max_integral = self.omega_max / max(self.ki, 1e-6)
            self.integral_error = max(-max_integral, min(max_integral, self.integral_error))
        
        # Derivative term
        if dt > 0:
            derivative = (error - self.last_error) / dt
        else:
            derivative = 0.0
        self.last_error = error
        
        # PID control law
        omega = self.kp * error + self.ki * self.integral_error + self.kd * derivative
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

        self.log(f"PID: e={error:.2f}, I={self.integral_error:.2f}, D={derivative:.2f}, ω={omega:.3f}, v_l={v_l:.2f}, v_r={v_r:.2f}")

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
        
        # Reset PID state
        self.integral_error = 0.0
        self.last_error = 0.0
        
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
