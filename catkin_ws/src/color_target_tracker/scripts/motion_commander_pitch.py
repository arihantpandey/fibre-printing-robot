#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String

# Initialize publishers
command_pub = rospy.Publisher('/actuator_motor_commands', String, queue_size=10)
base_command_pub = rospy.Publisher('/base_motor_commands', String, queue_size=10)

# Camera and image properties
image_width = 1280
image_height = 720
diagonal_fov = 55  # Logitech camera's diagonal FOV in degrees

# Center of the image
image_center_x = image_width / 2
image_center_y = image_height / 2

# Calculate Vertical Field of View (VFOV)
aspect_ratio = image_width / image_height
vfov = 2 * math.atan(math.tan(math.radians(diagonal_fov / 2)) / math.sqrt(aspect_ratio**2 + 1))
vfov_degrees = math.degrees(vfov)

# Pixels per degree calculation
pixels_per_degree = image_height / vfov_degrees

# Distance to the object (in meters)
distance_to_object = 0.3  # 30 cm for better understanding, adjust as needed

# Sensitivity factors for elevation and pitch adjustments
elevation_sensitivity = 2
pitch_sensitivity = 1.0

# Define alignment threshold
alignment_threshold = 80
# Define servo spin duration in seconds
servo_spin_duration = 5.0

def target_position_callback(data):
    # Calculate commands for camera orientation
    elevation, pitch = p_control_pitch(data.x, data.y, distance_to_object)

    if abs(elevation) < alignment_threshold:
        # Camera is aligned, move base first then trigger the servo to spin
        # base_command_pub.publish("move_to_distance:10")  
        # rospy.sleep(5)
        trigger_servo_spin(servo_spin_duration)

    else:
        # Publish camera orientation commands
        camera_command = "Elevation:{},Pitch:{}".format(elevation, pitch)
        rospy.loginfo(camera_command)
        command_pub.publish(camera_command)
        
        # calculate and publish base movement commands to keep the target in center
        calculate_base_movement(data.x, data.y)

def p_control_pitch(x, y, distance):
    # Calculate pitch based on vertical pixel offset, VFOV, and distance
    pixel_offset = y - image_center_y
    # Calculate the real-world vertical offset at the distance
    vertical_field_at_distance = 2 * distance * math.tan(math.radians(vfov_degrees / 2))
    real_world_offset = (pixel_offset / image_height) * vertical_field_at_distance
    angle_offset = math.degrees(math.atan(real_world_offset / distance))
    
    pitch_change = -angle_offset * pitch_sensitivity

    # Normalize the pitch to be within 0 to 90 degrees where 45 is neutral
    pitch = int(45 + pitch_change)
    pitch = max(0, min(90, pitch))

    elevation_change = -1 * (image_center_y - y) * elevation_sensitivity

    return elevation_change, pitch

def trigger_servo_spin(duration):
    servo_command = "SpinServo:{}".format(duration)
    command_pub.publish(servo_command)

def calculate_base_movement(x, y):
    x_offset = x - image_center_x
    
    # Assuming 1 second of strafing for every 200 pixels of error
    strafe_time = abs(x_offset) / 200.0
    strafe_direction = "strafe_left" if x_offset < 0 else "strafe_right"

    # Send strafe command with the calculated time
    strafe_command = "{}:{}".format(strafe_direction, strafe_time)
    rospy.loginfo(strafe_command)
    base_command_pub.publish(strafe_command)

    # Wait for the strafe to complete
    # rospy.sleep(strafe_time)

if __name__ == '__main__':
    rospy.init_node('motion_planner', anonymous=False)
    rospy.Subscriber("/color_target_position", Point, target_position_callback)
    rospy.spin()
