#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun lane_following_pkg wheel_control_node.py

# wait for app to end
dt-launchfile-join