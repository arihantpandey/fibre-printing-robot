#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <Servo.h>
#include <AccelStepper.h>

const int dirPin = 2;
const int stepPin = 3;
#define motorInterfaceType 1
AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);

const int upperLimitSwitchPin = 30;
const int lowerLimitSwitchPin = 31;
const int pumpPin = 50;
const int pitchServoPin = 12;
bool has_triggered = 0;
ros::NodeHandle nh;
Servo printServo;
Servo pitchServo;

int lastPitch = 45;

std_msgs::Bool upper_limit_msg;
std_msgs::Bool lower_limit_msg;

ros::Publisher upper_limit_pub("/actuator_upper_limit_switch", &upper_limit_msg);
ros::Publisher lower_limit_pub("/actuator_lower_limit_switch", &lower_limit_msg);

void applyPitch() {
    int clampedPitch = constrain(lastPitch, 10, 80);
    pitchServo.write(clampedPitch);
}

void checkAndMoveStepper(int elevation) {
    bool atUpperLimit = digitalRead(upperLimitSwitchPin) == LOW;
    bool atLowerLimit = digitalRead(lowerLimitSwitchPin) == LOW;
    if(atUpperLimit || atLowerLimit) { 
      has_triggered=1;
    }
    upper_limit_msg.data = has_triggered;
    lower_limit_msg.data = has_triggered;
    upper_limit_pub.publish(&upper_limit_msg);
    lower_limit_pub.publish(&lower_limit_msg);

    if ((elevation > 0 && !atUpperLimit) || (elevation < 0 && !atLowerLimit)) {
        myStepper.move(elevation);
        myStepper.runToPosition();
    } else {
        applyPitch(); 
        delay(2000);
    }
}

void motorCommandCallback(const std_msgs::String &cmd_msg) {
    String command = cmd_msg.data;
    if (command.startsWith("SpinServo:")) {
        digitalWrite(pumpPin, HIGH);
        String durationStr = command.substring(10);
        long duration = durationStr.toInt() * 1000;
        printServo.writeMicroseconds(1600);
        delay(duration);
        printServo.writeMicroseconds(1500);
        digitalWrite(pumpPin, LOW);
        
    } else if (command.startsWith("Elevation:")) {
        int commaIndex = command.indexOf(',');
        String elevationStr = command.substring(10, commaIndex);
        String pitchStr = command.substring(command.indexOf("Pitch:") + 6);

        
        nh.loginfo(("ElevationStr: " + elevationStr).c_str());
        nh.loginfo(("PitchStr: " + pitchStr).c_str());

        int elevation = elevationStr.toInt();
        lastPitch = pitchStr.toInt();  
        checkAndMoveStepper(elevation);
        nh.loginfo(("Received Elevation: " + String(elevation) + ", Pitch: " + String(lastPitch)).c_str());  
    }
}

ros::Subscriber<std_msgs::String> sub("/actuator_motor_commands", &motorCommandCallback);

void setup() {
    pinMode(upperLimitSwitchPin, INPUT);
    pinMode(lowerLimitSwitchPin, INPUT);
    pinMode(pumpPin, OUTPUT);
    digitalWrite(pumpPin, LOW);
    Serial.begin(57600);
    nh.initNode(); 
    nh.subscribe(sub);
    nh.advertise(upper_limit_pub);
    nh.advertise(lower_limit_pub);
    pitchServo.attach(pitchServoPin);
    pitchServo.write(45);
    printServo.attach(11);
    printServo.writeMicroseconds(1500);
    myStepper.setMaxSpeed(1000);
    myStepper.setAcceleration(100);
}

void loop() {
    nh.spinOnce();
    delay(10);  
}
