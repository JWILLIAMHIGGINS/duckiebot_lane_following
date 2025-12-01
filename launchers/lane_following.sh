#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

echo "Starting lane following with test camera..."

# Start test camera publisher in background
rosrun lane_following_pkg test_camera_publisher.py &

# Wait for publisher to start
sleep 2

# Start vision processing node
rosrun lane_following_pkg vision_processing_node.py

rosrun lane_following_pkg web_camera_node.py

# wait for app to end
dt-launchfile-join