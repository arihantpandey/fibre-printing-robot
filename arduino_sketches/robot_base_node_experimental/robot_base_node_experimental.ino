#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

// first motor driver
const int IN1_1 = 5;
const int IN2_1 = 4;
const int ENA_1 = 6;

const int IN3_1 = 8;
const int IN4_1 = 9;
const int ENB_1 = 7;

// second motor driver
const int IN1_2 = 26;
const int IN2_2 = 28;
const int ENA_2 = 12;

const int IN3_2 = 22;
const int IN4_2 = 24;
const int ENB_2 = 13;

const int trigPin = 44;
const int echoPin = 45;

int flag = 0;

std_msgs::Float32 distance_msg;
ros::Publisher pub_distance("/ultrasonic_distance", &distance_msg);
// Define the callback function for the command subscriber



float getDistance()
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH);
    return duration * 0.034 / 2;
}

void adjust_distance(float target_distance)
{
    while (abs(getDistance() - target_distance) > 2)
    { // Allow 2 cm tolerance
        if (getDistance() > target_distance)
        {
            All_Motors_Backward(100);
        }
        else
        {
            All_Motors_Forward(100);
        }
    }
    All_Motors_Brake();
}

void Motor_Forward(int inPin1, int inPin2, int enPin, int Speed)
{
    digitalWrite(inPin1, HIGH);
    digitalWrite(inPin2, LOW);
    analogWrite(enPin, Speed);
}

void Motor_Backward(int inPin1, int inPin2, int enPin, int Speed)
{
    digitalWrite(inPin1, LOW);
    digitalWrite(inPin2, HIGH);
    analogWrite(enPin, Speed);
}

void Motor_Brake(int inPin1, int inPin2)
{
    digitalWrite(inPin1, LOW);
    digitalWrite(inPin2, LOW);
}

void All_Motors_Forward(int Speed)
{
    Motor_Forward(IN1_1, IN2_1, ENA_1, Speed);
    Motor_Forward(IN3_1, IN4_1, ENB_1, Speed);
    Motor_Forward(IN1_2, IN2_2, ENA_2, Speed);
    Motor_Forward(IN3_2, IN4_2, ENB_2, Speed);
}

void All_Motors_Backward(int Speed)
{
    Motor_Backward(IN1_1, IN2_1, ENA_1, Speed);
    Motor_Backward(IN3_1, IN4_1, ENB_1, Speed);
    Motor_Backward(IN1_2, IN2_2, ENA_2, Speed);
    Motor_Backward(IN3_2, IN4_2, ENB_2, Speed);
}

void All_Motors_Brake()
{
    Motor_Brake(IN1_1, IN2_1);
    Motor_Brake(IN3_1, IN4_1);
    Motor_Brake(IN1_2, IN2_2);
    Motor_Brake(IN3_2, IN4_2);
}

void Strafe_Right(int Speed)
{
    Motor_Backward(IN1_1, IN2_1, ENA_1, Speed); // Front Left Backward
    Motor_Forward(IN3_1, IN4_1, ENB_1, Speed);  // Rear Left Forward
    Motor_Backward(IN3_2, IN4_2, ENB_2, Speed); // Rear Right Backward
    Motor_Forward(IN1_2, IN2_2, ENA_2, Speed);  // Front Right Forward
}

void Strafe_Left(int Speed)
{
    Motor_Forward(IN1_1, IN2_1, ENA_1, Speed);  // Front Left Forward
    Motor_Backward(IN3_1, IN4_1, ENB_1, Speed); // Rear Left Backward
    Motor_Forward(IN3_2, IN4_2, ENB_2, Speed);  // Rear Right Forward
    Motor_Backward(IN1_2, IN2_2, ENA_2, Speed); // Front Right Backward

    
}
void commandCallback(const std_msgs::String &cmd_msg)
{
    String command = cmd_msg.data;
    nh.loginfo(cmd_msg.data);
    if (command.startsWith("set_distance:"))
    {
        // Parse the distance value after "set_distance:"
        String target_distance = command.substring(13);
        float dist = target_distance.toFloat();
        adjust_distance(dist);
    }
    else if (command.startsWith("strafe_left:"))
    {
        String durationStr = command.substring(12);
        float duration = durationStr.toFloat() * 1000;
        nh.loginfo(("Strafing left: " + durationStr).c_str());
        Strafe_Left(40);
        delay(duration);
        All_Motors_Brake();
    }
    else if (command.startsWith("strafe_right:"))
    {
        String durationStr = command.substring(13);
        float duration = durationStr.toFloat() * 1000;
        nh.loginfo(("Strafing right: " + durationStr).c_str());
        Strafe_Right(50);
        delay(duration);
        All_Motors_Brake();
    }
}

// Subscribe to the command topic
ros::Subscriber<std_msgs::String> sub("/base_motor_commands", &commandCallback);

void setup()
{
    pinMode(IN1_1, OUTPUT);
    pinMode(IN2_1, OUTPUT);
    pinMode(ENA_1, OUTPUT);

    pinMode(IN3_1, OUTPUT);
    pinMode(IN4_1, OUTPUT);
    pinMode(ENB_1, OUTPUT);

    pinMode(IN1_2, OUTPUT);
    pinMode(IN2_2, OUTPUT);
    pinMode(ENA_2, OUTPUT);

    pinMode(IN3_2, OUTPUT);
    pinMode(IN4_2, OUTPUT);
    pinMode(ENB_2, OUTPUT);
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    
    nh.initNode();
    nh.advertise(pub_distance);
    nh.subscribe(sub);
}

void loop()
{
    if(!flag) {
        adjust_distance(50);
        flag = 1;
    }
     float distance = getDistance();
     distance_msg.data = distance;
     pub_distance.publish(&distance_msg);

    nh.spinOnce();
    delay(200);
}
