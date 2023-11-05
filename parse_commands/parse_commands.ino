/*
This arduino sketch is a basic implementation of the hardware control of the robot.
It accepts serial commands from the Jetson and commands the motors accordingly.
This must be updated to transmit camera data and implement feedback control.
*/

#include <Servo.h>

// Motor control pins definitions removed because they are not used in the code

// Speed definitions
#define SPEED 80
#define TURN_SPEED 60

// Motor Direction pins for four-wheel drive
#define RightMotorDirPin1 22
#define RightMotorDirPin2 24
#define LeftMotorDirPin1 26
#define LeftMotorDirPin2 28
#define RightMotorDirPin1B 5
#define RightMotorDirPin2B 6
#define LeftMotorDirPin1B 7
#define LeftMotorDirPin2B 8

// PWM Speed Control pins for four-wheel drive
#define speedPinR 9
#define speedPinL 10
#define speedPinRB 11
#define speedPinLB 12

// Gimbal servo pins
const int panServoPin = 13;  // Changed from 9 to prevent conflict with speedPinR
const int tiltServoPin = 14; // Changed from 10 to prevent conflict with speedPinL

// Servo objects
Servo panServo;
Servo tiltServo;

// Ultrasonic pins
const int triggerPin = 6; // or another pin if you prefer
const int echoPin = 7;

void setup()
{
    Serial.begin(9600); // Serial communication baud rate

    // Attach servos to their respective pins
    panServo.attach(panServoPin);
    tiltServo.attach(tiltServoPin);

    // Set all motor control pins to OUTPUT
    pinMode(RightMotorDirPin1, OUTPUT);
    pinMode(RightMotorDirPin2, OUTPUT);
    pinMode(LeftMotorDirPin1, OUTPUT);
    pinMode(LeftMotorDirPin2, OUTPUT);
    pinMode(RightMotorDirPin1B, OUTPUT);
    pinMode(RightMotorDirPin2B, OUTPUT);
    pinMode(LeftMotorDirPin1B, OUTPUT);
    pinMode(LeftMotorDirPin2B, OUTPUT);

    // Ultrasonic sensor setup
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);

    // Initialize PWM pins to OUTPUT
    pinMode(speedPinR, OUTPUT);
    pinMode(speedPinL, OUTPUT);
    pinMode(speedPinRB, OUTPUT);
    pinMode(speedPinLB, OUTPUT);

    // Initialize motors to a stopped state
    stop_Stop();

    // Center servos
    panServo.write(90);
    tiltServo.write(90);
}

void loop()
{
    // Process incoming serial commands
    if (Serial.available() > 0)
    {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if (command == "FORWARD")
        {
            go_advance(SPEED);
        }
        else if (command == "BACK")
        {
            go_back(SPEED);
        }
        else if (command == "LEFT")
        {
            countclockwise(TURN_SPEED);
        }
        else if (command == "RIGHT")
        {
            clockwise(TURN_SPEED);
        }
        else if (command == "STOP")
        {
            stop_Stop();
        }
        else if (command.startsWith("PAN:"))
        {
            int panAngle = command.substring(4).toInt();
            panServo.write(panAngle);
        }
        else if (command.startsWith("TILT:"))
        {
            int tiltAngle = command.substring(5).toInt();
            tiltServo.write(tiltAngle);
        }
        else if (command == "DISTANCE")
        {
            int distance = getDistance();
            Serial.println(distance); 
        }
    }
}

// get distance reading
int getDistance()
{
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);

    long duration = pulseIn(echoPin, HIGH);

    // Calculating the distance
    int distance = duration * 0.034 / 2;
    return distance;
}

/*motor control*/
void go_advance(int speed)
{
    RL_fwd(speed);
    RR_fwd(speed);
    FR_fwd(speed);
    FL_fwd(speed);
}
void go_back(int speed)
{
    RL_bck(speed);
    RR_bck(speed);
    FR_bck(speed);
    FL_bck(speed);
}
void right_shift(int speed_fl_fwd, int speed_rl_bck, int speed_rr_fwd, int speed_fr_bck)
{
    FL_fwd(speed_fl_fwd);
    RL_bck(speed_rl_bck);
    RR_fwd(speed_rr_fwd);
    FR_bck(speed_fr_bck);
}
void left_shift(int speed_fl_bck, int speed_rl_fwd, int speed_rr_bck, int speed_fr_fwd)
{
    FL_bck(speed_fl_bck);
    RL_fwd(speed_rl_fwd);
    RR_bck(speed_rr_bck);
    FR_fwd(speed_fr_fwd);
}

void left_turn(int speed)
{
    RL_bck(0);
    RR_fwd(speed);
    FR_fwd(speed);
    FL_bck(0);
}
void right_turn(int speed)
{
    RL_fwd(speed);
    RR_bck(0);
    FR_bck(0);
    FL_fwd(speed);
}
void left_back(int speed)
{
    RL_fwd(0);
    RR_bck(speed);
    FR_bck(speed);
    FL_fwd(0);
}
void right_back(int speed)
{
    RL_bck(speed);
    RR_fwd(0);
    FR_fwd(0);
    FL_bck(speed);
}
void clockwise(int speed)
{
    RL_fwd(speed);
    RR_bck(speed);
    FR_bck(speed);
    FL_fwd(speed);
}
void countclockwise(int speed)
{
    RL_bck(speed);
    RR_fwd(speed);
    FR_fwd(speed);
    FL_bck(speed);
}

void FR_fwd(int speed) // front-right wheel forward turn
{
    digitalWrite(RightMotorDirPin1, HIGH);
    digitalWrite(RightMotorDirPin2, LOW);
    analogWrite(speedPinR, speed);
}
void FR_bck(int speed) // front-right wheel backward turn
{
    digitalWrite(RightMotorDirPin1, LOW);
    digitalWrite(RightMotorDirPin2, HIGH);
    analogWrite(speedPinR, speed);
}
void FL_fwd(int speed) // front-left wheel forward turn
{
    digitalWrite(LeftMotorDirPin1, HIGH);
    digitalWrite(LeftMotorDirPin2, LOW);
    analogWrite(speedPinL, speed);
}
void FL_bck(int speed) // front-left wheel backward turn
{
    digitalWrite(LeftMotorDirPin1, LOW);
    digitalWrite(LeftMotorDirPin2, HIGH);
    analogWrite(speedPinL, speed);
}

void RR_fwd(int speed) // rear-right wheel forward turn
{
    digitalWrite(RightMotorDirPin1B, HIGH);
    digitalWrite(RightMotorDirPin2B, LOW);
    analogWrite(speedPinRB, speed);
}
void RR_bck(int speed) // rear-right wheel backward turn
{
    digitalWrite(RightMotorDirPin1B, LOW);
    digitalWrite(RightMotorDirPin2B, HIGH);
    analogWrite(speedPinRB, speed);
}
void RL_fwd(int speed) // rear-left wheel forward turn
{
    digitalWrite(LeftMotorDirPin1B, HIGH);
    digitalWrite(LeftMotorDirPin2B, LOW);
    analogWrite(speedPinLB, speed);
}
void RL_bck(int speed) // rear-left wheel backward turn
{
    digitalWrite(LeftMotorDirPin1B, LOW);
    digitalWrite(LeftMotorDirPin2B, HIGH);
    analogWrite(speedPinLB, speed);
}

void stop_Stop() // Stop
{
    analogWrite(speedPinLB, 0);
    analogWrite(speedPinRB, 0);
    analogWrite(speedPinL, 0);
    analogWrite(speedPinR, 0);
}
