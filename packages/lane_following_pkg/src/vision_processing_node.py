#!/usr/bin/env python3
# Node for detecting lanes in images from the Duckiebot
# and publishing vanishing and middle point for usage in lane controller

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np

class Vision_Processing_Node(DTROS):
    def __init__(self, node_name):
        super(Vision_Processing_Node, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)

        self._vehicle_name = os.environ.get('VEHICLE_NAME', 'deutschbot')
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._latest_jpeg = None

        # Subscribe to camera
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self._on_image, queue_size=1)

        self.log("Vision processing node initialized")

    def _on_image(self, msg):
        self._latest_jpeg = bytes(msg.data)

        self.log("Image received")

        # Image preprocessing


if __name__ == '__main__':
    node = Vision_Processing_Node("vision_processing_node")

    rospy.spin()
