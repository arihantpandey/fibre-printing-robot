#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String

command_pub = rospy.Publisher('/arduino/motor_commands', String, queue_size=10)

# dimensions of the image
image_width = 640  
image_height = 480  

# Center of the image
image_center_x = image_width / 2
image_center_y = image_height / 2

# Sensitivity factors for elevation and pitch adjustments
# These need to be calibrated 
elevation_sensitivity = 1.0
pitch_sensitivity = 1.0

def target_position_callback(data):
    # Convert position to motion commands
    elevation = calculate_elevation(data.y)
    pitch = calculate_pitch(data.x)
    
    # Construct command string
    command = "Elevation:{},Pitch:{}".format(elevation, pitch)
    
    # Publish command to Arduino
    command_pub.publish(command)
    rospy.loginfo("Sending command: %s" % command)

def calculate_elevation(y):
    # Calculate elevation adjustment based on target's y position
    # Positive adjustment means moving up, negative means moving down
    elevation_error = image_center_y - y
    elevation_adjustment = elevation_error * elevation_sensitivity
    return elevation_adjustment

def calculate_pitch(x):
    # Calculate pitch adjustment based on target's x position
    # Positive adjustment means tilting right, negative means tilting left
    pitch_error = image_center_x - x
    pitch_adjustment = pitch_error * pitch_sensitivity
    return pitch_adjustment


if __name__ == '__main__':
    rospy.init_node('motion_commander', anonymous=True)
    rospy.Subscriber("/color_target/position", Point, target_position_callback)
    rospy.spin()
