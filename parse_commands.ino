#include <Servo.h>

// Motor control pins
const int motorA1 = 2;
const int motorA2 = 3;
const int motorB1 = 4;
const int motorB2 = 5;

// Gimbal servos
Servo panServo;
Servo tiltServo;
const int panServoPin = 9;
const int tiltServoPin = 10;

void setup() {
  Serial.begin(115200);

  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

  panServo.attach(panServoPin);
  tiltServo.attach(tiltServoPin);
  
  // Initialize motors and servos to a safe state
  stopMotors();
  panServo.write(90); 
  tiltServo.write(90);
}

void loop() {
  if (Serial.available() > 0) {
    // Read the incoming command
    String command = Serial.readStringUntil('\n');
    command.trim();

    // Parse the command and execute actions
    if (command == "FORWARD") {
      moveForward();
    } else if (command == "BACK") {
      moveBackward();
    } else if (command == "LEFT") {
      turnLeft();
    } else if (command == "RIGHT") {
      turnRight();
    } else if (command == "STOP") {
      stopMotors();
    } else if (command.startsWith("PAN:")) {
      int panAngle = command.substring(4, command.indexOf(';')).toInt();
      panServo.write(panAngle);
    } else if (command.startsWith("TILT:")) {
      int tiltAngle = command.substring(5).toInt();
      tiltServo.write(tiltAngle);
    }
  }
}

void moveForward() {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
}

void moveBackward() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
}

void turnLeft() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
}

void turnRight() {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
}

void stopMotors() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
}
