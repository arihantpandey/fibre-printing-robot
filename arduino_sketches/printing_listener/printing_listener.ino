#include <ros.h>
#include <std_msgs/String.h>
#include <Servo.h>
#include <AccelStepper.h>

ros::NodeHandle nh;

// Initialize the servo for pitch control
Servo pitchServo;

// Initialize the stepper for elevation control
// AccelStepper stepper(interfaceType, stepPin, dirPin);
// For example, using driver with STEP and DIRECTION pin interface
AccelStepper stepper(AccelStepper::DRIVER, 2, 3);

void motorCommandCallback(const std_msgs::String& cmd_msg) {
  String command = cmd_msg.data;
  int separatorIndex = command.indexOf(',');

  String elevationStr = command.substring(10, separatorIndex);
  String pitchStr = command.substring(separatorIndex + 7);

  // Convert strings to integers
  int elevation = elevationStr.toInt();
  int pitch = pitchStr.toInt();

  // Placeholder for converting elevation to stepper steps
  // Assuming 60cm max elevation corresponds to a certain number of steps (e.g., 2000 steps)
  long steps = map(elevation, 0, 600, 0, 2000); // Map 0-60cm to 0-2000 steps
  stepper.moveTo(steps);
  stepper.runToPosition();

  // Placeholder for converting pitch to servo angle
  // Servo expects values from 0 to 180, so map -45 to 45 degrees to 0 to 180
  int servoAngle = map(pitch, -45, 45, 0, 180);
  pitchServo.write(servoAngle);

  Serial.print("Elevation steps: ");
  Serial.print(steps);
  Serial.print(", Pitch angle: ");
  Serial.println(servoAngle);
}

ros::Subscriber<std_msgs::String> sub("arduino/motor_commands", &motorCommandCallback);

void setup() {
  Serial.begin(9600);
  nh.initNode();
  nh.subscribe(sub);

  // Setup servo
  pitchServo.attach(9); // Attach the servo on pin 9 to the servo object

  // Setup stepper
  stepper.setMaxSpeed(1000); // Set max speed, adjust as necessary
  stepper.setAcceleration(500); // Set acceleration, adjust as necessary
}

void loop() {
  nh.spinOnce();
  delay(10);
}
