#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from std_msgs.msg import String

class ColorTargetDetector:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.target_pub = rospy.Publisher("/color_target/position", Point, queue_size=10)
        self.command_sub = rospy.Subscriber("/arduino/motor_commands", String, self.command_callback)
        self.bridge = CvBridge()
        self.last_command = ""  # Attribute to store the last motor command


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

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1]

        for contour in contours:
            # Calculate bounding box
            x, y, w, h = cv2.boundingRect(contour)
            # Draw bounding box
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)

            # Calculate centroid of the bounding box
            cx = x + w//2
            cy = y + h//2
            self.target_pub.publish(Point(x=cx, y=cy, z=0))

        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(cv_image, "Cmd: {}".format(self.last_command), (10, 30), font, 1, (0, 0, 255), 2, cv2.LINE_AA)

        # Display the resulting frame
        cv2.imshow('Color Target Detector', cv_image)
        cv2.waitKey(1)  # Add a small delay so the window can update
    
    def command_callback(self, msg):
        self.last_command = msg.data  # Update the last command

if __name__ == '__main__':
    rospy.init_node('color_detector', anonymous=True)
    od = ColorTargetDetector()
    rospy.spin()
    cv2.destroyAllWindows()  # Make sure to destroy any OpenCV windows when the node is stopped
