#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

echo "Starting lane following..."

# Start vision processing node in background
rosrun lane_following_pkg vision_processing_node.py &

# Start lane following controller node in background
rosrun lane_following_pkg lane_controller_node.py

# Start web camera viewer (runs in foreground)
#rosrun lane_following_pkg web_camera_node.py

# wait for app to end
dt-launchfile-join