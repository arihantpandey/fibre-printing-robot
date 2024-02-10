#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point

class ColorTargetDetector:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        self.target_pub = rospy.Publisher("/color_target/position", Point, queue_size=10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Convert BGR to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define range of orange color in HSV
        lower_orange = np.array([5, 50, 50])
        upper_orange = np.array([15, 255, 255])

        # Threshold the HSV image to get only orange colors
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # Find contours and take the largest one as the target
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1]

        if contours:
            # Assume the largest contour is the target.
            M = cv2.moments(contours[0])
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                self.target_pub.publish(Point(x=cx, y=cy, z=0))

if __name__ == '__main__':
    rospy.init_node('color_detector', anonymous=True)
    od = ColorTargetDetector()
    rospy.spin()
