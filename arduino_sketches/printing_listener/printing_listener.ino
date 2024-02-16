#include <ros.h>
#include <std_msgs/String.h>
#include <Servo.h>
#include <AccelStepper.h>


// Define pin connections
const int dirPin = 2;
const int stepPin = 3;

// Define motor interface type
#define motorInterfaceType 1
// Creates an instance
AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);

ros::NodeHandle nh;

// Initialize the servo for pitch control
Servo pitchServo;


void motorCommandCallback(const std_msgs::String& cmd_msg) {
  
  String command = cmd_msg.data;
  int separatorIndex = command.indexOf(',');

  String elevationStr = command.substring(10, separatorIndex);
  String pitchStr = command.substring(separatorIndex + 7);

  // Convert strings to integers
  int elevation = elevationStr.toInt();
  int pitch = pitchStr.toInt();
  

  if (elevation != 0) {
    Serial.println("Received command: ");
  Serial.println(elevation);
        // Move a fixed small amount up or down
        myStepper.move(elevation * 200); // 50 can be changed based on your step resolution
        while(myStepper.distanceToGo() != 0) {
            myStepper.run();
        }
    }

  // Placeholder for converting pitch to servo angle
  // Servo expects values from 0 to 180, so map -45 to 45 degrees to 0 to 180
  int servoAngle = map(pitch, -45, 45, 0, 180);
  pitchServo.write(servoAngle);

  Serial.print("Elevation direction: ");
  Serial.print(elevation);
  Serial.print(", Pitch angle: ");
  Serial.println(servoAngle);
}

ros::Subscriber<std_msgs::String> sub("arduino/motor_commands", &motorCommandCallback);

void setup() {
  Serial.begin(57600);
  nh.initNode();
  nh.subscribe(sub);

  // Setup servo
  pitchServo.attach(9); // Attach the servo on pin 9 to the servo object

  // Setup stepper
  myStepper.setMaxSpeed(1000);
  myStepper.setAcceleration(100);
  myStepper.setSpeed(500);
}

void loop() {
  nh.spinOnce();
  delay(10);
}
