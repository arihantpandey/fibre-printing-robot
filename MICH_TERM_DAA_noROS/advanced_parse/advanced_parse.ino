#include <Servo.h>

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

Servo myServo;
byte servoPin = 8;
byte servoMin = 10;
byte servoMax = 170;
byte servoPos = 0;
byte newServoPos = servoMin;

const byte numLEDs = 2;
byte ledPin[numLEDs] = {12, 13};
unsigned long LEDinterval[numLEDs] = {200, 400};
unsigned long prevLEDmillis[numLEDs] = {0, 0};
const int triggerPin = 2;
const int echoPin = 3;
const byte buffSize = 40;
char inputBuffer[buffSize];
const char startMarker = '<';
const char endMarker = '>';
byte bytesRecvd = 0;
boolean readInProgress = false;
boolean newDataFromPC = false;

char messageFromPC[buffSize] = {0};
int newFlashInterval = 0;
float servoFraction = 0.0; // fraction of servo range to move

unsigned long curMillis;

unsigned long prevReplyToPCmillis = 0;
unsigned long replyToPCinterval = 500;

//=============

void setup()
{
    Serial.begin(14400);

    delay(500); // delay() is OK in setup as it only happens once

    // initialize the servo
    myServo.attach(servoPin);
    // moveServo();

    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);

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

    // tell the PC we are ready
    Serial.println("<Arduino is ready>");
}

//=============

void loop()
{
    curMillis = millis();
    getDataFromPC();
    commMotors();
    // updateFlashInterval();
    // updateServoPos();
    // replyToPC();
    // Reset messageFromPC after handling the command
    // memset(messageFromPC, 0, sizeof(messageFromPC));
    // flashLEDs();
    // moveServo();

    if (millis() - prevReplyToPCmillis >= replyToPCinterval)
    {
        prevReplyToPCmillis = millis();
        sendDistanceToPC();
    }
}

//=============
void sendDistanceToPC()
{
    int distance = measureDistance();
    Serial.print("<Distance ");
    Serial.print(distance);
    Serial.println(">");
}
int measureDistance()
{
    // Clears the triggerPin
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);

    // Sets the triggerPin on HIGH state for 10 micro seconds
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);

    // Reads the echoPin, returns the sound wave travel time in microseconds
    long duration = pulseIn(echoPin, HIGH);

    // Calculating the distance
    int distance = duration * 0.034 / 2; // Speed of sound wave divided by 2
    return distance;
}
void getDataFromPC()
{

    // receive data from PC and save it into inputBuffer

    if (Serial.available() > 0)
    {

        char x = Serial.read();

        // the order of these IF clauses is significant

        if (x == endMarker)
        {
            readInProgress = false;
            newDataFromPC = true;
            inputBuffer[bytesRecvd] = 0;
            parseData();
        }

        if (readInProgress)
        {
            // Serial.print("Reading: "); // Debug print
            // Serial.println(x);
            inputBuffer[bytesRecvd] = x;
            bytesRecvd++;
            if (bytesRecvd == buffSize)
            {
                bytesRecvd = buffSize - 1;
            }
        }

        if (x == startMarker)
        {
            bytesRecvd = 0;
            readInProgress = true;
        }
    }
}

//=============

void parseData()
{

    // split the data into its parts

    char *strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(inputBuffer, ","); // get the first part - the string
    strcpy(messageFromPC, strtokIndx);     // copy it to messageFromPC

    strtokIndx = strtok(NULL, ",");      // this continues where the previous call left off
    newFlashInterval = atoi(strtokIndx); // convert this part to an integer

    strtokIndx = strtok(NULL, ",");
    servoFraction = atof(strtokIndx); // convert this part to a float
}

//=============

void replyToPC()
{

    if (newDataFromPC)
    {
        newDataFromPC = false;
        // Serial.print("<Msg ");
        // Serial.print(messageFromPC);
        // Serial.print(" NewFlash ");
        // Serial.print(newFlashInterval);
        // Serial.print(" SrvFrac ");
        // Serial.print(servoFraction);
        // Serial.print(" SrvPos ");
        // Serial.print(newServoPos);
        // Serial.print(" Time ");
        // Serial.print(curMillis >> 9); // divide by 512 is approx = half-seconds
        // Serial.println(">");
        Serial.print("<command ");
        Serial.print(messageFromPC);
        Serial.println(">");
    }
}

void commMotors()
{
    // if (messageFromPC)
    // {
    //     Serial.print("<command ");
    //     Serial.print(messageFromPC);
    //     Serial.println(">");
    // }
    if (strcmp(messageFromPC, "FORWARD") == 0)
    {
        // Serial.println("<command FORWARD>");
        go_advance(SPEED);
    }
    else if (strcmp(messageFromPC, "BACKWARD") == 0)
    {
        // Serial.println("<command BACKWARD>");
        go_back(SPEED);
    }
    else if (strcmp(messageFromPC, "LEFT") == 0)
    {
        // Serial.println("<command LEFT>");
        countclockwise(TURN_SPEED);
    }
    else if (strcmp(messageFromPC, "RIGHT") == 0)
    {
        // Serial.println("<command RIGHT>");
        clockwise(TURN_SPEED);
    }
    else if (strcmp(messageFromPC, "LSHIFT") == 0)
    {
        // Serial.println("<command BACKWARD>");
        left_shift(SPEED, SPEED, SPEED, SPEED);
    }
    else if (strcmp(messageFromPC, "RSHIFT") == 0)
    {
        // Serial.println("<command BACKWARD>");
        right_shift(SPEED, SPEED, SPEED, SPEED);
    }
    else if (strcmp(messageFromPC, "STOP") == 0)
    {
        // Serial.println("<command STOP>");
        stop_Stop();
    }
}

//============

void updateFlashInterval()
{

    // this illustrates using different inputs to call different functions
    if (strcmp(messageFromPC, "LED1") == 0)
    {
        updateLED1();
    }

    if (strcmp(messageFromPC, "LED2") == 0)
    {
        updateLED2();
    }
}

//=============

void updateLED1()
{

    if (newFlashInterval > 100)
    {
        LEDinterval[0] = newFlashInterval;
    }
}

//=============

void updateLED2()
{

    if (newFlashInterval > 100)
    {
        LEDinterval[1] = newFlashInterval;
    }
}

//=============

void flashLEDs()
{

    for (byte n = 0; n < numLEDs; n++)
    {
        if (curMillis - prevLEDmillis[n] >= LEDinterval[n])
        {
            prevLEDmillis[n] += LEDinterval[n];
            digitalWrite(ledPin[n], !digitalRead(ledPin[n]));
        }
    }
}

//=============

void updateServoPos()
{

    byte servoRange = servoMax - servoMin;
    if (servoFraction >= 0 && servoFraction <= 1)
    {
        newServoPos = servoMin + ((float)servoRange * servoFraction);
    }
}

//=============

void moveServo()
{
    if (servoPos != newServoPos)
    {
        servoPos = newServoPos;
        myServo.write(servoPos);
    }
}

/*motor control*/
void go_advance(int speed)
{
    RL_bck(speed);
    RR_bck(speed);
    FR_bck(speed);
    FL_bck(speed);
}
void go_back(int speed)
{
    RL_fwd(speed);
    RR_fwd(speed);
    FR_fwd(speed);
    FL_fwd(speed);
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