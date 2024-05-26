// Include the AccelStepper Library
#include <AccelStepper.h>

const int dirPin = 2;
const int stepPin = 3;

#define motorInterfaceType 1

// Creates an instance
AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);

void setup() {
  myStepper.setMaxSpeed(1000);
  myStepper.setAcceleration(50);
  myStepper.setSpeed(200);
  myStepper.moveTo(200);
}

void loop() {
  if (myStepper.distanceToGo() == 0) 
    myStepper.moveTo(-myStepper.currentPosition());

  // Move the motor one step
  myStepper.run();
}
