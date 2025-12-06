#!/usr/bin/env python3
"""
SDTS node implementing the Scenario-Driven Transition System from the paper,
compatible with your LaneControllerNode.

Drop into your ROS workspace and run alongside the lane controller and vision nodes.
"""

import os
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import Float64

class SDTSNode(object):
    def __init__(self, node_name='sdts_node'):
        rospy.init_node(node_name, anonymous=False)

        # Vehicle name
        self._vehicle_name = os.environ.get('VEHICLE_NAME', 'deutschbot')

        # Controller service names (must match lane controller)
        self.reset_srv_name = f"/{self._vehicle_name}/lane_controller/reset"
        self.enable_srv_name = f"/{self._vehicle_name}/lane_controller/enable"
        self.disable_srv_name = f"/{self._vehicle_name}/lane_controller/disable"

        # Wait for controller services if available (try but continue if not up yet)
        rospy.loginfo("SDTS: waiting for controller services (short timeout)...")
        try:
            rospy.wait_for_service(self.reset_srv_name, timeout=5.0)
            rospy.wait_for_service(self.enable_srv_name, timeout=5.0)
            rospy.wait_for_service(self.disable_srv_name, timeout=5.0)
            self.reset_srv = rospy.ServiceProxy(self.reset_srv_name, Empty)
            self.enable_srv = rospy.ServiceProxy(self.enable_srv_name, Empty)
            self.disable_srv = rospy.ServiceProxy(self.disable_srv_name, Empty)
            rospy.loginfo("SDTS: Controller services available.")
        except Exception as e:
            rospy.logwarn(f"SDTS: controller services not available at startup: {e}")
            self.reset_srv = None
            self.enable_srv = None
            self.disable_srv = None

        # SDTS parameters (tunable)
        self.v_nominal = rospy.get_param('~v_nominal', rospy.get_param('~v', 0.005))
        self.v_turn = rospy.get_param('~v_turn', 0.002)
        self.x_lost_thresh = rospy.get_param('~x_lost_thresh', 0.5)    # units consistent with x_v
        self.t_lost_timeout = rospy.get_param('~t_lost_timeout', 1.0)
        self.t_recover_timeout = rospy.get_param('~t_recover_timeout', 4.0)
        self.recenter_thresh = rospy.get_param('~recenter_thresh', 0.05)
        self.t_stable = rospy.get_param('~t_stable', 0.5)

        # Publishers: we will publish wheels commands for Recovery if needed
        self.wheels_pub = rospy.Publisher(f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1)
        # Also observe omega published by controller (optional)
        self.omega_sub = rospy.Subscriber(f"/{self._vehicle_name}/lane_controller/omega", Float64, lambda msg: None)

        # Subscriptions (observations)
        self.x_v = None
        self.x_m = None
        self.corner = False
        self.corner_dir = "none"
        self.last_seen = rospy.Time.now()
        self.lines_visible = 'none'  # 'both', 'one', 'none'
        rospy.Subscriber(f"/{self._vehicle_name}/lane_following/vanishing_point", Point, self.cb_vanish)
        rospy.Subscriber(f"/{self._vehicle_name}/lane_following/mid_point", Point, self.cb_mid)
        rospy.Subscriber(f"/{self._vehicle_name}/lane_following/corner_detected", Bool, self.cb_corner)
        # corner_direction is optional (vision node may publish it)
        rospy.Subscriber(f"/{self._vehicle_name}/lane_following/corner_direction", String, self.cb_corner_dir)

        # Internal state
        self.state = 'Idle'
        self.state_enter_time = rospy.Time.now()
        self.last_lost_time = None

        # timer for periodic checks
        self.timer = rospy.Timer(rospy.Duration(0.05), self.tick)  # 20Hz

        rospy.loginfo("SDTS node initialized (state=Idle).")

    # ----------------- Callers for controller services -----------------
    def call_reset(self):
        if self.reset_srv:
            try:
                self.reset_srv()
                rospy.loginfo("SDTS: called controller reset")
            except Exception as e:
                rospy.logwarn(f"SDTS: reset service call failed: {e}")

    def call_enable(self):
        if self.enable_srv:
            try:
                self.enable_srv()
                rospy.loginfo("SDTS: called controller enable")
            except Exception as e:
                rospy.logwarn(f"SDTS: enable service call failed: {e}")

    def call_disable(self):
        if self.disable_srv:
            try:
                self.disable_srv()
                rospy.loginfo("SDTS: called controller disable")
            except Exception as e:
                rospy.logwarn(f"SDTS: disable service call failed: {e}")

    # ----------------- Callbacks -----------------
    def cb_vanish(self, msg):
        # msg.x expected to match controller's coordinate convention (centered or pixel)
        self.x_v = msg.x
        self.last_seen = rospy.Time.now()
        # simple visibility heuristics
        if self.x_m is not None:
            self.lines_visible = 'both'
        else:
            self.lines_visible = 'one'

    def cb_mid(self, msg):
        self.x_m = msg.x
        self.last_seen = rospy.Time.now()
        if self.x_v is not None:
            self.lines_visible = 'both'

    def cb_corner(self, msg):
        prev = self.corner
        self.corner = bool(msg.data)
        if self.corner and not prev:
            rospy.loginfo("SDTS: corner detected")
        if not self.corner and prev:
            rospy.loginfo("SDTS: corner cleared")

    def cb_corner_dir(self, msg):
        self.corner_dir = msg.data if msg is not None else "none"

    # ----------------- Recovery primitive -----------------
    def recovery_spin_step(self, duration=0.1):
        """Publish a small rotation wheel command for duration seconds."""
        # choose small angular command: rotate in place slowly
        # simple symmetric left/right wheel speeds in rad/s
        # use default turn speed that respects wheel_speed_max param from controller
        v_turn_ang = 0.5  # rad/s angular per wheel (tunable)
        cmd = WheelsCmdStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.vel_left = -v_turn_ang
        cmd.vel_right = v_turn_ang
        self.wheels_pub.publish(cmd)
        # sleep short while to let it act (non-blocking small sleep)
        # (But the timer loop will run again; avoid long sleeps.)

    # ----------------- tick / main logic -----------------
    def tick(self, event):
        now = rospy.Time.now()

        # compute lost predicate
        lost = (now - self.last_seen).to_sec() > self.t_lost_timeout

        # compute absolute x deviation (if None treat as large)
        if self.x_v is None:
            x_abs = float('inf')
        else:
            x_abs = abs(self.x_v)

        # state transitions
        if self.state == 'Idle':
            # auto-enable behavior: move to LaneFollowing by enabling controller
            rospy.loginfo_throttle(10, "SDTS: Idle -> enabling and entering LaneFollowing")
            # ensure controller uses nominal speed
            rospy.set_param('~v', self.v_nominal)
            self.call_enable()
            self.call_reset()
            self.state = 'LaneFollowing'
            self.state_enter_time = now
            return

        if self.state == 'LaneFollowing':
            if self.corner:
                # begin corner maneuver
                rospy.loginfo("SDTS: LaneFollowing -> CornerManeuver")
                rospy.set_param('~v', self.v_turn)
                self.call_reset()
                # allow controller to handle corner (controller sets constant omega when corner_detected)
                self.state = 'CornerManeuver'
                self.state_enter_time = now
                return
            if lost or x_abs > self.x_lost_thresh:
                # enter recovery
                rospy.loginfo("SDTS: LaneFollowing -> Recovery (lost lines)")
                self.call_disable()   # disable PID-driven outputs
                self.state = 'Recovery'
                self.state_enter_time = now
                self.last_lost_time = now
                return

        if self.state == 'CornerManeuver':
            # If corner cleared and we have good lines, go back to lane following
            if (not self.corner) and (self.lines_visible == 'both') and (self.x_v is not None) and (abs(self.x_v) <= self.recenter_thresh):
                # require stability for t_stable seconds
                if (now - self.state_enter_time).to_sec() >= self.t_stable:
                    rospy.loginfo("SDTS: CornerManeuver -> LaneFollowing (cleared)")
                    rospy.set_param('~v', self.v_nominal)
                    self.call_reset()
                    self.call_enable()
                    self.state = 'LaneFollowing'
                    self.state_enter_time = now
                    return
            # if lines lost during corner for long time -> Recovery
            if (now - self.state_enter_time).to_sec() > self.t_recover_timeout and ((now - self.last_seen).to_sec() > self.t_lost_timeout):
                rospy.loginfo("SDTS: CornerManeuver -> Recovery (lost)")
                self.call_disable()
                self.state = 'Recovery'
                self.state_enter_time = now
                self.last_lost_time = now
                return

        if self.state == 'Recovery':
            # perform simple recovery behaviour: spin slowly and check for lines
            # If we detect lines again and stable, re-enable controller
            if self.lines_visible == 'both' and (self.x_v is not None) and (abs(self.x_v) <= self.recenter_thresh):
                # require stability
                if (now - self.last_seen).to_sec() < self.t_lost_timeout:
                    # recovered
                    rospy.loginfo("SDTS: Recovery -> LaneFollowing (recovered)")
                    rospy.set_param('~v', self.v_nominal)
                    self.call_reset()
                    self.call_enable()
                    self.state = 'LaneFollowing'
                    self.state_enter_time = now
                    return
            # If still in recovery, publish small spin commands periodically
            self.recovery_spin_step()
            # also timeout after t_recover_timeout to escalate (could be emergency stop)
            if (now - self.last_lost_time).to_sec() > self.t_recover_timeout:
                rospy.logwarn("SDTS: Recovery timeout exceeded (consider manual intervention)")

        if self.state == 'Disabled':
            # stay disabled until operator re-enables
            pass

        if self.state == 'Stopped':
            # do nothing
            pass

    # ----------------- run -----------------
def main():
    node = SDTSNode()
    rospy.spin()

if __name__ == "__main__":
    main()
