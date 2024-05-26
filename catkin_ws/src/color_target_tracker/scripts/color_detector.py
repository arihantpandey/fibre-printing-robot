#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import yaml
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from std_msgs.msg import String
from std_msgs.msg import Float32  
import dynamic_reconfigure.server
from color_target_tracker.cfg import ColorRangeConfig

class ColorTargetDetector:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.server = dynamic_reconfigure.server.Server(ColorRangeConfig, self.config_callback)
        self.target_pub = rospy.Publisher("/color_target_position", Point, queue_size=10)
        self.actuator_command_sub = rospy.Subscriber("/actuator_motor_commands", String, self.command_callback)
        self.base_command_sub = rospy.Subscriber("/base_motor_commands", String, self.base_command_callback)
        self.distance_sub = rospy.Subscriber("/ultrasonic_distance", Float32, self.distance_callback)
        self.bridge = CvBridge()
        self.last_actuator_command = "" 
        self.last_base_command = ""      
        self.last_distance = 0.0         

    def load_hsv_range(self):
        script_dir = os.path.dirname(os.path.realpath(__file__))
        package_path = os.path.join(script_dir, '..')
        hsv_range_path = os.path.join(package_path, 'cfg', 'hsv_range.yaml')
        try:
            with open(hsv_range_path, 'r') as file:
                hsv_range = yaml.safe_load(file)
                return np.array(hsv_range['lower']), np.array(hsv_range['upper'])
        except Exception as e:
            rospy.logerr("Failed to load HSV range from file: {}".format(e))
            return np.array([5, 100, 100]), np.array([15, 255, 255])

    def config_callback(self, config, level):
        rospy.loginfo("Reconfiguring HSV range")
        self.lower_orange = np.array([config.lower_hue, config.lower_saturation, config.lower_value])
        self.upper_orange = np.array([config.upper_hue, config.upper_saturation, config.upper_value])
        return config

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_orange, self.upper_orange)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1]

        if contours:
            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cx = x + w//2
                cy = y + h//2
                self.target_pub.publish(Point(x=cx, y=cy, z=0))
        else:
            self.target_pub.publish(Point(x=0, y=0, z=0))

        cv2.putText(cv_image, "Actuator Cmd: {}".format(self.last_actuator_command), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(cv_image, "Base Cmd: {}".format(self.last_base_command), (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(cv_image, "Distance: {:.2f} cm".format(self.last_distance), (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

        cv2.imshow('Color Target Detector', cv_image)
        cv2.waitKey(3)

    def command_callback(self, msg):
        self.last_actuator_command = msg.data

    def base_command_callback(self, msg):
        self.last_base_command = msg.data

    def distance_callback(self, msg):
        self.last_distance = msg.data  

if __name__ == '__main__':
    rospy.init_node('image_processor', anonymous=False)
    ct_detector = ColorTargetDetector()
    rospy.spin()
    cv2.destroyAllWindows()
