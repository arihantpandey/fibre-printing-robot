#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <Servo.h>

ros::NodeHandle nh;

Servo elevationServo;  // Servo for camera elevation

// Placeholder function for setting up Mecanum wheel control
// This should initialize motors or motor drivers
void setupMecanumWheels() {
  // Initialize Mecanum wheel motors here
}

// Function to control Mecanum wheels based on Twist messages
// This needs to be implemented based on your motor driver and wheel setup
void controlMecanumWheels(const geometry_msgs::Twist& cmd_msg) {
  // Extract linear and angular velocities
  float linear_x = cmd_msg.linear.x;
  float angular_z = cmd_msg.angular.z;

  // Convert these velocities into wheel speeds
  // The conversion depends on your specific robot configuration
  // Example: 
  // float wheel1Speed = linear_x + angular_z;
  // float wheel2Speed = linear_x - angular_z;
  // Adjust speeds for each wheel based on cmd_msg
  
  // Control the wheels with the calculated speeds
  // Example: setWheelSpeed(1, wheel1Speed);
}

void cameraCommandCallback(const std_msgs::String& cmd_msg) {
  // Example of parsing the camera elevation command
  String command = cmd_msg.data.c_str(); // Convert to String for easier handling
  int elevationStart = command.indexOf("Elevation:") + 10;
  int pitchStart = command.indexOf(",Pitch:");
  String elevationStr = command.substring(elevationStart, pitchStart);
  
  int elevation = elevationStr.toInt();
  // Map the elevation value to your servo's range
  // Example assumes elevation is an angle; adjust if your input differs
  elevationServo.write(elevation);
}

void baseCommandCallback(const geometry_msgs::Twist& cmd_msg) {
  controlMecanumWheels(cmd_msg);
}

ros::Subscriber<std_msgs::String> camera_sub("/arduino/motor_commands", cameraCommandCallback);
ros::Subscriber<geometry_msgs::Twist> base_sub("/robot_base/motor_commands", baseCommandCallback);

void setup() {
  nh.initNode();
  nh.subscribe(camera_sub);
  nh.subscribe(base_sub);
  
  elevationServo.attach(9); // Attach the servo to a digital pin
  setupMecanumWheels(); // Setup Mecanum wheels
}

void loop() {
  nh.spinOnce();
  delay(10);
}
