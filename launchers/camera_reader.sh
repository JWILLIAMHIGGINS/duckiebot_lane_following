#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

rostopic echo /deutschbot/camera_node/image/compressed &

# launch subscriber
rosrun lane_following_pkg camera_reader_node.py

# wait for app to end
dt-launchfile-join