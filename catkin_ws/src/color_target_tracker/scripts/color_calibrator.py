#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import dynamic_reconfigure.client
import yaml
import os

class ColorCalibrator:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.bridge = CvBridge()
        self.hsv_range = {"lower": np.array([0, 0, 0]), "upper": np.array([0, 0, 0])}
        self.client = dynamic_reconfigure.client.Client("color_calibrator", timeout=30, config_callback=self.config_callback)
        
        rospy.on_shutdown(self.save_hsv_range_to_file)

    def config_callback(self, config):
        self.hsv_range["lower"] = [config.lower_hue, config.lower_saturation, config.lower_value]
        self.hsv_range["upper"] = [config.upper_hue, config.upper_saturation, config.upper_value]

    def save_hsv_range_to_file(self):
        # Construct the file path to save in the cfg directory
        script_dir = os.path.dirname(os.path.realpath(__file__))  # Absolute path to the current script
        cfg_dir_path = os.path.join(script_dir, '..', 'cfg')  # Navigate up to the parent, then to the cfg directory
        cfg_dir_path = os.path.abspath(cfg_dir_path)  # Get the absolute path to the cfg directory
        
        # Ensure the cfg directory exists
        if not os.path.exists(cfg_dir_path):
            os.makedirs(cfg_dir_path)
        
        file_path = os.path.join(cfg_dir_path, 'hsv_range.yaml')  # Full path to the hsv_range.yaml file

        # Write the HSV range to the YAML file
        with open(file_path, 'w') as file:
            yaml.dump(self.hsv_range, file, default_flow_style=False)
        rospy.loginfo("HSV range saved to %s" % file_path)


    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        
        if self.hsv_range:
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.hsv_range["lower"], self.hsv_range["upper"])
            cv2.imshow("Mask", mask)
            cv2.waitKey(3)

if __name__ == '__main__':
    rospy.init_node('color_calibrator', anonymous=True)
    calibrator = ColorCalibrator()
    rospy.spin()
