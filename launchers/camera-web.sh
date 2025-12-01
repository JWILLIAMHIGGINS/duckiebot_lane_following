#!/bin/bash

source /environment.sh
dt-launchfile-init

echo "Starting camera test with web viewer..."
echo "Open: http://localhost:8080"

# Start publisher
rosrun lane_following_pkg test_camera_publisher.py &

# Wait for publisher
sleep 2

# Start web viewer
rosrun lane_following_pkg web_camera_node.py

dt-launchfile-join
