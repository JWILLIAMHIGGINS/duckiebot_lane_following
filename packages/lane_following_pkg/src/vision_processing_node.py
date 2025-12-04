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

def find_line_intersection(line_1, line_2):
    """
        Find intersection point of two lines given in polar coordinates
    """
    r_1, theta_1 = line_1[0], line_1[1]
    a_white = -1*np.cos(theta_1)/np.sin(theta_1)
    b_white = r_1/np.sin(theta_1)

    r_2, theta_2 = line_2[0], line_2[1]
    a_yellow = -1*np.cos(theta_2)/np.sin(theta_2)
    b_yellow = r_2/np.sin(theta_2)

    intersec_x = (b_white - b_yellow) / (a_yellow - a_white)
    intersec_y = a_white * intersec_x + b_white
    return (intersec_x, intersec_y)


class Vision_Processing_Node(DTROS):
    def __init__(self, node_name):
        super(Vision_Processing_Node, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)

        self._vehicle_name = os.environ.get('VEHICLE_NAME', 'deutschbot')
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._latest_jpeg = None
        
        # Bridge between OpenCV and ROS
        self._bridge = CvBridge()

        # Subscribe to camera (compressed)
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self._on_image, queue_size=1)

        # Publish debug image (note: topic name WITHOUT /compressed suffix)
        # ROS will automatically add /compressed when you subscribe
        self.pub_debug = rospy.Publisher(
            f"/{self._vehicle_name}/lane_detection/debug/image/compressed", 
            CompressedImage, 
            queue_size=10
        )

        self.log("Vision processing node initialized")

    def _on_image(self, msg):
        #self.log("Image received")
        
        # Convert compressed image to CV image (one simple line!)
        image = self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')


        # === Image preprocessing ===

        # Gaussian filtering
        image = cv2.GaussianBlur(image, (3,3), 0)


        # === Extract colors ===
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # White color
        white_lower_bound = np.array([0, 0, int(0.70*255)])             # [H_min, S_min, V_min]
        white_upper_bound = np.array([255, int(0.20*255), 255])         # [H_max, S_max, V_max]
        white_mask = cv2.inRange(image_hsv, white_lower_bound, white_upper_bound)
        image_white_part = cv2.bitwise_and(image, image, mask=white_mask)
        # Convert to grayscale
        _, _, image_grayscale_white = cv2.split(image_white_part)

        # Yellow color
        yellow_lower_bound = np.array([20, int(0.30*255), int(0.60*255)])             # [H_min, S_min, V_min]
        yellow_upper_bound = np.array([45, 255, 255])                     # [H_max, S_max, V_max]
        yellow_mask = cv2.inRange(image_hsv, yellow_lower_bound, yellow_upper_bound)
        image_yellow_part = cv2.bitwise_and(image, image, mask=yellow_mask)
        # Convert to grayscale
        _, _, image_grayscale_yellow = cv2.split(image_yellow_part)


        # === Edge detection ===
        edges_white = cv2.Canny(image_grayscale_white,100,200)
        edges_yellow = cv2.Canny(image_grayscale_yellow,100,200)
        

        # === ROI Selection ===
        # Create mask
        img_height, img_width = edges_white.shape
        ROI_mask = np.zeros(edges_white.shape, np.uint8)
        ROI_mask[img_height//2:img_height, 0:img_width] = 1
        # Apply mask
        image_white_cropped = cv2.bitwise_and(edges_white, edges_white, mask=ROI_mask)
        image_yellow_cropped = cv2.bitwise_and(edges_yellow, edges_yellow, mask=ROI_mask)

        # Convert to BGR for visualization
        image_grayscale_BGR = cv2.cvtColor(image_white_cropped+image_yellow_cropped, cv2.COLOR_GRAY2BGR)


        # === Line Detection ===
        # Hough Transform
        white_lines = cv2.HoughLines(image_white_cropped, 1, np.pi/180, 70, None, 0, 0)
        yellow_lines = cv2.HoughLines(image_yellow_cropped, 1, np.pi/180, 40, None, 0, 0)

        if white_lines is not None and yellow_lines is not None:
            # Calc average lines
            white_line_avrg = (np.mean(white_lines[:, 0, 0]), np.mean(white_lines[:, 0, 1]))        # (r, theta)
            yellow_line_avrg = (np.mean(yellow_lines[:, 0, 0]), np.mean(yellow_lines[:, 0, 1]))        # (r, theta)

            # Find vanishing point as intersection of guidelines
            vanishing_point = find_line_intersection(white_line_avrg, yellow_line_avrg)

            # Find midpoint as center of intersection of guidelines with abscissa
            abscissa = (img_height, np.pi/2)            # (y = 0*x + img_height)
            intersec_wh_absc = find_line_intersection(white_line_avrg, abscissa)
            intersec_yl_absc = find_line_intersection(yellow_line_avrg, abscissa)
            midpoint = (int( np.mean([intersec_wh_absc[0], intersec_yl_absc[0]]) ), abscissa[0])     # Calculate mean of the two x-coordinates


        # Draw lines
        for lines in [white_lines, yellow_lines]:
            if lines is not None:
                r_avrg = np.mean(lines[:, 0, 0])
                theta_avrg = np.mean(lines[:, 0, 1])

                a = np.cos(theta_avrg)
                b = np.sin(theta_avrg)
                x0 = a * r_avrg
                y0 = b * r_avrg
                pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                cv2.line(image_grayscale_BGR, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)

                
                for i in range(0, len(lines)):
                    rho = lines[i][0][0]
                    theta = lines[i][0][1]
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                    pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                    cv2.line(image_grayscale_BGR, pt1, pt2, (0,0,120), 1, cv2.LINE_AA)
                
        if white_lines is not None and yellow_lines is not None:
            # Draw vanishing point
            if(img_height > vanishing_point[0] >= 0 and img_width > vanishing_point[1] >= 0):
                cv2.circle(image_grayscale_BGR, (int(vanishing_point[0]), int(vanishing_point[1])), 5, (0,255,0), -1)
            
            # Draw midpoint
            if(img_height > midpoint[0] >= 0):
                cv2.circle(image_grayscale_BGR, (int(midpoint[0]), int(midpoint[1])), 15, (0,255,0), -1)

        # Debug: Convert grayscale image back to CompressedImage
        debug_msg = self._bridge.cv2_to_compressed_imgmsg(image_grayscale_BGR)
        self.pub_debug.publish(debug_msg)



if __name__ == '__main__':
    node = Vision_Processing_Node("vision_processing_node")

    rospy.spin()
