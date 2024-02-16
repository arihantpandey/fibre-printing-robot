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

int tol = 75;
void motorCommandCallback(const std_msgs::String &cmd_msg)
{

    String command = cmd_msg.data;
    int separatorIndex = command.indexOf(',');

    String elevationStr = command.substring(10, separatorIndex);
    String pitchStr = command.substring(separatorIndex + 7);

    // Convert strings to integers
    int elevation = elevationStr.toInt();
    int pitch = pitchStr.toInt();

    if (abs(elevation) > tol)
    {
        myStepper.move(elevation);
        while (myStepper.distanceToGo() != 0)
        {
            myStepper.run();
        }
    }
}

ros::Subscriber<std_msgs::String> sub("arduino/motor_commands", &motorCommandCallback);

void setup()
{
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

void loop()
{
    nh.spinOnce();
    delay(10);
}
