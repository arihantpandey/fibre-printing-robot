#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Point
from std_msgs.msg import String, Bool, Float32

# Initialize publishers
command_pub = rospy.Publisher('/actuator_motor_commands', String, queue_size=10)
base_command_pub = rospy.Publisher('/base_motor_commands', String, queue_size=10)

# Camera and image properties
image_width = 1280
image_height = 720
diagonal_fov = 55  # Logitech camera's diagonal FOV in degrees

# Center of the image
image_center_x =  -50+ image_width / 2 
image_center_y = -150 + image_height / 2

# Calculate Vertical and Horizontal Field of View (VFOV and HFOV)
aspect_ratio = image_width / image_height
vfov = 2 * math.atan(math.tan(math.radians(diagonal_fov / 2)) / math.sqrt(aspect_ratio**2 + 1))
vfov_degrees = math.degrees(vfov)
hfov = 2 * math.atan(aspect_ratio * math.tan(math.radians(diagonal_fov / 2)))
hfov_degrees = math.degrees(hfov)

# Pixels per degree calculation
pixels_per_degree_vfov = image_height / vfov_degrees
pixels_per_degree_hfov = image_width / hfov_degrees

# Distance to the object (in meters)
distance_to_object = 30  # Initial value, will be updated from the ultrasonic sensor

# Sensitivity factors for elevation and pitch adjustments
elevation_sensitivity = 5
pitch_sensitivity = 1 

# Define alignment threshold
alignment_threshold = 70
# Define servo spin duration in seconds
servo_spin_duration = 5.0

lateral_tolerance = 0.07  # 7 cm

# State variables
stage = 1  # 1: initial distance, 2: 30 cm distance, 3: 15 cm distance
lateral_flag = False
switch_triggered = False
last_detected = False  # Tracks if the last frame had a detected contour

def target_position_callback(data):
    global distance_to_object
    global stage
    global lateral_flag
    global switch_triggered
    global last_detected

    if data.x == 0 and data.y == 0 and not last_detected:
        # First attempt with no contour detected: adjust pitch to 60 degrees
        command_pub.publish("Elevation:0,Pitch:55")
        last_detected = True

    # Calculate commands for camera orientation
    elevation, pitch = p_control_pitch(data.x, data.y, distance_to_object)

    if switch_triggered:
        base_command_pub.publish("set_distance:12")
        # rospy.sleep(3)
        while True:     
            trigger_servo_spin(servo_spin_duration)
    elif stage == 1 and lateral_flag:
        if not switch_triggered and abs(elevation) < alignment_threshold:
            # Move to 30 cm
            base_command_pub.publish("set_distance:30")
            rospy.sleep(3)
            stage = 2
            # lateral_flag = False
        elif not switch_triggered:
            # Adjust actuator
            camera_command = "Elevation:{},Pitch:{}".format(-elevation, pitch)
            rospy.loginfo(camera_command)
            command_pub.publish(camera_command)
    elif stage == 2 and lateral_flag:
        if not switch_triggered and abs(elevation) < alignment_threshold:
            # Move to 15 cm
            base_command_pub.publish("set_distance:12")
            rospy.sleep(3)
            stage = 3
            # lateral_flag = False
        elif not switch_triggered:
            # Adjust actuator
            camera_command = "Elevation:{},Pitch:{}".format(-elevation, pitch)
            rospy.loginfo(camera_command)
            command_pub.publish(camera_command)
    elif stage == 3 and lateral_flag:
        if abs(elevation) < alignment_threshold:
            # Spin the servo
            trigger_servo_spin(servo_spin_duration)
    else:
        # Calculate and publish base movement commands to keep the target in center
        lateral_flag = calculate_base_movement(data.x, data.y)

# # Lateral tolerance (in meters)

# lateral_flag = False
# enhanced = False
# def target_position_callback(data):
#     global distance_to_object
#     global lateral_flag
#     global enhanced
#     global switch_triggered
#     # Calculate commands for camera orientation
#     elevation, pitch = p_control_pitch(data.x, data.y, distance_to_object)

#     if enhanced and lateral_flag and (abs(elevation) < alignment_threshold):
#         # Camera is aligned, move base first then trigger the servo to spin
        
#         base_command_pub.publish("set_distance:15")  
#         rospy.sleep(3)
#         trigger_servo_spin(servo_spin_duration) 
#         # enhanced=False
#     elif lateral_flag and (abs(elevation) < alignment_threshold):
#         base_command_pub.publish("set_distance:30")
#         rospy.sleep(3)
#         enhanced = True
#         elevation, pitch = p_control_pitch(data.x, data.y, 0.3)
#         rospy.loginfo(camera_command)
#         command_pub.publish(camera_command)
#         rospy.sleep(3)
#     elif lateral_flag:
#         # Publish camera orientation commands
#         camera_command = "Elevation:{},Pitch:{}".format(elevation, pitch)
#         rospy.loginfo(camera_command)
#         command_pub.publish(camera_command)
#     else:
#         # Calculate and publish base movement commands to keep the target in center
#         lateral_flag = calculate_base_movement(data.x, data.y)

def p_control_pitch(x, y, distance):
    # Calculate pitch based on vertical pixel offset, VFOV, and distance
    image_center_y2= -50 + image_height / 2
    pixel_offset = y - image_center_y2
    # Calculate the real-world vertical offset at the distance
    vertical_field_at_distance = 2 * distance * math.tan(math.radians(vfov_degrees / 2))
    real_world_offset = ((y - image_height/2) / image_height) * vertical_field_at_distance
    angle_offset = math.degrees(math.atan(real_world_offset / distance))
    
    pitch_change = -angle_offset * pitch_sensitivity*1.0

    # Normalize the pitch to be within 0 to 90 degrees where 45 is neutral
    pitch = int(45 + pitch_change)
    pitch = max(0, min(90, pitch))

    elevation_change = -1 * (image_center_y - y) * elevation_sensitivity

    return elevation_change, pitch

def trigger_servo_spin(duration):
    servo_command = "SpinServo:{}".format(duration)
    command_pub.publish(servo_command)

def calculate_base_movement(x, y):
    global distance_to_object
    x_offset = x - image_center_x
    
    # Calculate the real-world horizontal distance at the given distance
    horizontal_field_at_distance = 2 * distance_to_object * math.tan(math.radians(hfov_degrees / 2))
    # Calculate the pixel offset tolerance for 7 cm (lateral tolerance)
    pixel_offset_tolerance = (lateral_tolerance / horizontal_field_at_distance) * image_width
    
    if abs(x_offset) > pixel_offset_tolerance:
        # Assuming 1 second of strafing for every 200 pixels of error
        strafe_time = abs(x_offset) / 800.0
        strafe_direction = "strafe_left" if x_offset < 0 else "strafe_right"

        # Send strafe command with the calculated time
        strafe_command = "{}:{}".format(strafe_direction, strafe_time)
        if strafe_time < 0.05:
            return True
        rospy.loginfo(strafe_command)
        base_command_pub.publish(strafe_command)
        return False
        # Wait for the strafe to complete
        # rospy.sleep(strafe_time)

def upper_limit_switch_callback(data):
    rospy.loginfo("Upper limit switch status: {}".format(data.data))
    global switch_triggered
    if data.data:
        switch_triggered = True

def lower_limit_switch_callback(data):
    rospy.loginfo("Lower limit switch status: {}".format(data.data))
    global switch_triggered
    if data.data:
        switch_triggered = True

def ultrasonic_sensor_callback(data):
    global distance_to_object
    distance_to_object = data.data  # Update the distance to the object

if __name__ == '__main__':
    rospy.init_node('motion_planner', anonymous=False)
    rospy.Subscriber("/color_target_position", Point, target_position_callback)
    rospy.Subscriber("/actuator_upper_limit_switch", Bool, upper_limit_switch_callback)
    rospy.Subscriber("/actuator_lower_limit_switch", Bool, lower_limit_switch_callback)
    rospy.Subscriber("/ultrasonic_distance", Float32, ultrasonic_sensor_callback)
    rospy.sleep(3)
    rospy.spin()
