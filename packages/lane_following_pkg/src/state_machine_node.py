#!/usr/bin/env python3

# JOSEPH

#!/usr/bin/env python3
import os
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from std_msgs.msg import Float64

class SDTS_Node(object):
    def __init__(self, name='sdts_node'):
        rospy.init_node(name)
        self._vehicle_name = os.environ.get('VEHICLE_NAME', 'deutschbot')
        # Parameters (including thresholds/timeouts)
        self.t_lost = rospy.get_param('~t_lost', 1.0)
        self.t_recover = rospy.get_param('~t_recover', 4.0)
        self.t_stable = rospy.get_param('~t_stable', 0.5)
        self.x_lost_thresh = rospy.get_param('~x_lost_thresh', 0.5)  # normalized

        # Controller param names (read for visibility / future usage)
        self.kp = rospy.get_param('~kp', 0.001)
        self.ki = rospy.get_param('~ki', 0.0005)
        self.kd = rospy.get_param('~kd', 0.00001)
        self.v = rospy.get_param('~v', 0.005)

        # Services to call on lane_controller
        self.reset_srv_name = f"/{self._vehicle_name}/lane_controller/reset"
        self.enable_srv_name = f"/{self._vehicle_name}/lane_controller/enable"
        self.disable_srv_name = f"/{self._vehicle_name}/lane_controller/disable"

        # Prepare service proxies (may block until exists; catch exceptions)
        rospy.wait_for_service(self.reset_srv_name, timeout=5.0)
        rospy.wait_for_service(self.enable_srv_name, timeout=5.0)
        rospy.wait_for_service(self.disable_srv_name, timeout=5.0)
        self.reset_srv = rospy.ServiceProxy(self.reset_srv_name, Empty)
        self.enable_srv = rospy.ServiceProxy(self.enable_srv_name, Empty)
        self.disable_srv = rospy.ServiceProxy(self.disable_srv_name, Empty)

        # Subscriptions (observations)
        self.x_v = None
        self.x_m = None
        self.corner = False
        self.last_valid_time = None
        self.last_seen = rospy.Time.now()
        self.lines_visible = 'none'

        rospy.Subscriber(f"/{self._vehicle_name}/lane_following/vanishing_point", Point, self.cb_vanish)
        rospy.Subscriber(f"/{self._vehicle_name}/lane_following/mid_point", Point, self.cb_mid)
        rospy.Subscriber(f"/{self._vehicle_name}/lane_following/corner_detected", Bool, self.cb_corner)

        # Optionally monitor published omega
        self.omega_sub = rospy.Subscriber(f"/{self._vehicle_name}/lane_controller/omega", Float64, lambda msg: None)

        # Internal state
        self.state = 'Idle'   # Idle at startup
        self.state_enter_time = rospy.Time.now()

        self.timer = rospy.Timer(rospy.Duration(0.05), self.tick)  # 20 Hz tick

    def cb_vanish(self, msg):
        # Assuming incoming x is centered (controller sets it to center coords)
        self.x_v = msg.x
        self.last_seen = rospy.Time.now()
        # update lines_visible heuristics
        if msg.x is None:
            self.lines_visible = 'none'
        else:
            # assume if mid_point present also then both
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
            rospy.loginfo("SDTS: corner detected -> switching logic may disable PID")
        if not self.corner and prev:
            rospy.loginfo("SDTS: corner cleared")

    def call_reset(self):
        try:
            self.reset_srv()
        except Exception as e:
            rospy.logwarn("Reset service failed: %s" % e)

    def call_enable(self):
        try:
            self.enable_srv()
        except Exception as e:
            rospy.logwarn("Enable service failed: %s" % e)

    def call_disable(self):
        try:
            self.disable_srv()
        except Exception as e:
            rospy.logwarn("Disable service failed: %s" % e)

    def tick(self, _event):
        now = rospy.Time.now()

        # Update derived predicates
        # lost detection
        if (now - self.last_seen).to_sec() > self.t_lost:
            lost = True
        else:
            lost = False

        # checks for large deviation; x_v is expected to be centered coords
        if self.x_v is None:
            x_abs = float('inf')
        else:
            x_abs = abs(self.x_v)

        # State machine
        if self.state == 'Idle':
            # wait for enable_by_user (we interpret presence of enable service call as external)
            # For demo, automatically enable after startup:
            rospy.loginfo_throttle(10, "SDTS in Idle: auto-enabling lane controller")
            self.call_enable()
            self.call_reset()
            self.state = 'LaneFollowing'
            self.state_enter_time = now

        elif self.state == 'LaneFollowing':
            if self.corner:
                rospy.loginfo("SDTS: transition to CornerManeuver")
                # rely on lane_controller to use corner behavior; maybe adjust v param externally
                self.state = 'CornerManeuver'
                self.state_enter_time = now
            elif lost or x_abs > self.x_lost_thresh:
                rospy.loginfo("SDTS: lost lines -> Recovery")
                # optionally disable controller and take explicit recovery commands
                self.call_disable()
                self.state = 'Recovery'
                self.state_enter_time = now

        elif self.state == 'CornerManeuver':
            if not self.corner and self.lines_visible == 'both':
                rospy.loginfo("SDTS: corner cleared -> re-enable PID")
                self.call_reset()
                self.call_enable()
                self.state = 'LaneFollowing'
                self.state_enter_time = now
            elif (now - self.state_enter_time).to_sec() > self.t_recover and lost:
                rospy.loginfo("SDTS: corner + lost -> Recovery")
                self.call_disable()
                self.state = 'Recovery'
                self.state_enter_time = now

        elif self.state == 'Recovery':
            # In Recovery we perform simple behavior: try to reacquire for t_stable seconds
            if self.lines_visible == 'both' and abs(self.x_v) < 0.2:
                # recovered: re-enable controller
                rospy.loginfo("SDTS: recovered -> re-enable")
                self.call_reset()
                self.call_enable()
                self.state = 'LaneFollowing'
                self.state_enter_time = now

        elif self.state == 'Disabled':
            pass

        elif self.state == 'Stopped':
            pass

def main():
    node = SDTS_Node()
    rospy.spin()

if __name__ == "__main__":
    main()
