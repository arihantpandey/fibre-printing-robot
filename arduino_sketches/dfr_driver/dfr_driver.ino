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

const int trigPin = 46;
const int echoPin = 47;

void setup() {
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
    
    Serial.begin(9600);
}

void loop() {
    Strafe_Left(100);
    delay(2000);
    All_Motors_Brake();
    Strafe_Right(100);
    delay(2000);
    All_Motors_Brake();
}

float getDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH);

    float distance = duration * 0.034 / 2;
    return distance;
}

void Motor_Forward(int inPin1, int inPin2, int enPin, int Speed) {
    digitalWrite(inPin1, HIGH);
    digitalWrite(inPin2, LOW);
    analogWrite(enPin, Speed);
}

void Motor_Backward(int inPin1, int inPin2, int enPin, int Speed) {
    digitalWrite(inPin1, LOW);
    digitalWrite(inPin2, HIGH);
    analogWrite(enPin, Speed);
}

void Motor_Brake(int inPin1, int inPin2) {
    digitalWrite(inPin1, LOW);
    digitalWrite(inPin2, LOW);
}

void All_Motors_Forward(int Speed) {
    Motor_Forward(IN1_1, IN2_1, ENA_1, Speed);
    Motor_Forward(IN3_1, IN4_1, ENB_1, Speed);
    Motor_Forward(IN1_2, IN2_2, ENA_2, Speed);
    Motor_Forward(IN3_2, IN4_2, ENB_2, Speed);
}

void All_Motors_Backward(int Speed) {
    Motor_Backward(IN1_1, IN2_1, ENA_1, Speed);
    Motor_Backward(IN3_1, IN4_1, ENB_1, Speed);
    Motor_Backward(IN1_2, IN2_2, ENA_2, Speed);
    Motor_Backward(IN3_2, IN4_2, ENB_2, Speed);
}

void All_Motors_Brake() {
    Motor_Brake(IN1_1, IN2_1);
    Motor_Brake(IN3_1, IN4_1);
    Motor_Brake(IN1_2, IN2_2);
    Motor_Brake(IN3_2, IN4_2);
}

void Strafe_Right(int Speed) {
    float speed = Speed;
    Motor_Forward(IN1_1, IN2_1, ENA_1, speed*1.2); // Front Left Forward
    Motor_Backward(IN3_1, IN4_1, ENB_1, speed*1.3); // Rear Left Backward
    Motor_Forward(IN3_2, IN4_2, ENB_2, speed); // Rear Right Forward
    Motor_Backward(IN1_2, IN2_2, ENA_2, speed); // Front Right Backward
}

void Strafe_Left(int Speed) {
    float speed = Speed;
    Motor_Backward(IN1_1, IN2_1, ENA_1, speed*1.2); // Front Left Backward
    Motor_Forward(IN3_1, IN4_1, ENB_1, speed*1.3); // Rear Left Forward
    Motor_Backward(IN3_2, IN4_2, ENB_2, speed); // Rear Right Backward
    Motor_Forward(IN1_2, IN2_2, ENA_2, speed); // Front Right Forward
}
