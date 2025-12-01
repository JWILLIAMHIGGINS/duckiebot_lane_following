#!/usr/bin/env python3

# STEFFEN

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np

class TestCameraPublisher(DTROS):
    """Publishes test images without text overlay"""

    def __init__(self, node_name):
        super(TestCameraPublisher, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        self._vehicle_name = os.environ.get('VEHICLE_NAME', 'deutschbot')
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._bridge = CvBridge()
        self.pub = rospy.Publisher(self._camera_topic, CompressedImage, queue_size=1)
        
        self.log(f"Publishing to: {self._camera_topic}")
        rospy.sleep(1.0)

    def run(self):
        rate = rospy.Rate(30)
        frame = 0
        
        while not rospy.is_shutdown():
            # Create test image
            image = self._create_test_image(frame)
            
            # Publish
            msg = self._bridge.cv2_to_compressed_imgmsg(image, dst_format='jpeg')
            msg.header.stamp = rospy.Time.now()
            self.pub.publish(msg)
            
            frame += 1
            rate.sleep()
    
    def _create_test_image(self, frame):
        """Generate colorful test pattern"""
        width, height = 640, 480
        image = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Gradient background
        for y in range(height):
            for x in range(width):
                image[y, x] = [
                    int((x / width) * 255),
                    int((y / height) * 255),
                    int(((x + y) / (width + height)) * 255)
                ]
        
        # Moving circle
        cx = int(width / 2 + 150 * np.sin(frame * 0.05))
        cy = int(height / 2 + 100 * np.cos(frame * 0.05))
        cv2.circle(image, (cx, cy), 50, (255, 255, 255), -1)
        
        # Lane lines
        cv2.line(image, (100, height), (width//2 - 50, height//2), (255, 255, 0), 3)
        cv2.line(image, (width - 100, height), (width//2 + 50, height//2), (255, 255, 0), 3)
        
        return image

if __name__ == '__main__':
    node = TestCameraPublisher(node_name='test_camera_publisher')
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
