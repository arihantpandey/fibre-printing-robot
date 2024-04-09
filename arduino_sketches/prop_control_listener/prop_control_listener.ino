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

// Initialize the servo for pitch control (not used in this snippet but initialized)
Servo pitchServo;
// Initialize the continuous rotation servo for spinning action
Servo printServo;

int tol = 75; // Tolerance for the elevation check
bool hasServoSpun = false;

void motorCommandCallback(const std_msgs::String &cmd_msg) {
    String command = cmd_msg.data;
    int separatorIndex = command.indexOf(',');

    String elevationStr = command.substring(10, separatorIndex);
    // Convert strings to integers
    int elevation = elevationStr.toInt();

    // Move stepper motor if needed
    if (abs(elevation) > tol && !hasServoSpun) { // Check if the servo hasn't spun yet
        myStepper.move(elevation);
        while (myStepper.distanceToGo() != 0) {
            myStepper.run();
        }
    }

    // Check for the SpinServo command and ensure it hasn't spun yet
    if (command.indexOf("SpinServo:") != -1 && !hasServoSpun) {
        int spinIndex = command.indexOf("SpinServo:") + 10;
        String spinDurationStr = command.substring(spinIndex);
        long servoSpinDuration = spinDurationStr.toInt() * 1000; // Convert to milliseconds

        // Start spinning the printServo
        printServo.writeMicroseconds(1600); // Start spinning
        
        // Block further execution while spinning
        unsigned long startSpinTime = millis();
        while (millis() - startSpinTime < servoSpinDuration) {
            // Just wait here until the spin duration has elapsed
        }

        // Stop the printServo after the delay
        printServo.writeMicroseconds(1500);

        Serial.println("Servo spin complete.");
        
        // Set the flag to true so it doesn't spin again
//        hasServoSpun = true;
    }
}          


ros::Subscriber<std_msgs::String> sub("arduino/motor_commands", &motorCommandCallback);

void setup() {
    Serial.begin(57600);
    nh.initNode();
    nh.subscribe(sub);

    // Setup servos
    pitchServo.attach(9); // not used yet but setup
    printServo.attach(10); // Continuous rotation servo for printing/spinning action
    printServo.writeMicroseconds(1500);

    // Setup stepper
    myStepper.setMaxSpeed(1000);
    myStepper.setAcceleration(100);
    myStepper.setSpeed(500);
    bool hasServoSpun = false;
}

void loop() {
    nh.spinOnce();
    delay(10);
}
