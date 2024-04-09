#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String

# Initialize publishers
command_pub = rospy.Publisher('/arduino/motor_commands', String, queue_size=10)
base_command_pub = rospy.Publisher('/robot_base/motor_commands', Twist, queue_size=10)

# dimensions of the image
image_width = 1280  
image_height = 720

# Center of the image
image_center_x = image_width / 2
image_center_y = image_height / 2

# Sensitivity factors for elevation and pitch adjustments
# These need to be calibrated 
elevation_sensitivity = 2
pitch_sensitivity = 1.0

# Define an alignment threshold
alignment_threshold = 80
# Define servo spin duration in seconds
servo_spin_duration = 2.0


def target_position_callback(data):
    # Calculate commands for camera orientation
    # elevation, pitch = calculate_camera_orientation(data.x, data.y)
    elevation,pitch = p_control_elevation(data.x, data.y)

    if abs(elevation) < alignment_threshold:
        # Camera is aligned, trigger the servo to spin
        trigger_servo_spin(servo_spin_duration)
        rospy.loginfo("SPINNING")
    else:
            # Publish camera orientation commands
        camera_command = "Elevation:{},Pitch:{}".format(elevation, pitch)
        command_pub.publish(camera_command)
        
        # Calculate and publish base movement commands
        base_command = calculate_base_movement(data.x, data.y)
        base_command_pub.publish(base_command)
        rospy.loginfo("Camera Cmd: %s | Base Cmd: Linear: %s, Angular: %s",
                  camera_command, base_command.linear.x, base_command.angular.z)

def calculate_camera_orientation(x, y):
    # Define a tolerance value (1/3 of the image dimensions)
    width_tolerance = image_width / 3
    height_tolerance = image_height / 4
    
    # Initialize elevation and pitch changes to zero
    elevation_change = 0
    pitch_change = 0

    # Calculate error from the center
    elevation_error = image_center_y - y
    pitch_error = image_center_x - x

    # Check if the target is outside the middle third of the screen
    if elevation_error > height_tolerance:
        # Target is above the middle third, move the actuator down
        elevation_change = -1
    elif elevation_error < -height_tolerance:
        # Target is below the middle third, move the actuator up
        elevation_change = 1
    
    # Assuming pitch control is not needed for this example
    # If it was needed, you could implement a similar logic based on width_tolerance

    return elevation_change, pitch_change

def p_control_elevation(x,y):
    elevation_error = image_center_y - y

    elevation_change = -elevation_error * elevation_sensitivity
    
    return elevation_change, 0

def trigger_servo_spin(duration):
    # Construct the command to spin the servo
    servo_command = "SpinServo:{}".format(duration)
    # Publish the servo command
    command_pub.publish(servo_command)

def calculate_base_movement(x, y):
    command = Twist()
    
    # Simplified logic to orient and approach the target
    if x < image_width / 4:
        command.angular.z = 0.5  # Rotate left
    elif x > 3 * image_width / 4:
        command.angular.z = -0.5  # Rotate right
    else:
        command.angular.z = 0  # No rotation needed if target is centered
    
    if y < image_height / 4:
        command.linear.x = 0.5  # Move forward
    elif y > 3 * image_height / 4:
        command.linear.x = -0.5  # Move backward
    else:
        command.linear.x = 0  # Stop moving if at an ideal distance
    
    # Adjust these values based on your robot's specific capabilities and testing
    return command

if __name__ == '__main__':
    rospy.init_node('motion_commander', anonymous=True)
    rospy.Subscriber("/color_target/position", Point, target_position_callback)
    rospy.spin()
