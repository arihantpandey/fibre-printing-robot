#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <Servo.h>

ros::NodeHandle nh;

Servo elevationServo;

// Placeholder function for setting up Mecanum wheel control
void setupMecanumWheels() {
  
}

// Function to control Mecanum wheels based on Twist messages
void controlMecanumWheels(const geometry_msgs::Twist& cmd_msg) {
  // Extract linear and angular velocities
  float linear_x = cmd_msg.linear.x;
  float angular_z = cmd_msg.angular.z;

  // Convert these velocities into wheel speeds
}

void cameraCommandCallback(const std_msgs::String& cmd_msg) {
  String command = cmd_msg.data.c_str(); 
  int elevationStart = command.indexOf("Elevation:") + 10;
  int pitchStart = command.indexOf(",Pitch:");
  String elevationStr = command.substring(elevationStart, pitchStart);
  
  int elevation = elevationStr.toInt();
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
  
  elevationServo.attach(9); 
  setupMecanumWheels(); 
}
void loop() {
  nh.spinOnce();
  delay(10);
}
