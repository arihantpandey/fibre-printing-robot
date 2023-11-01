const int En = 2;
const int MS1 = 3;
const int MS2 = 4;
const int MS3 = 5;
const int RST = 6;
const int SLP = 7; 
const int StepPin = 8;
const int DirPin = 9;

const int delaytime = 1000;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  // Set as 1/16 step resolution 
  pinMode(En, OUTPUT);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);
  pinMode(RST, OUTPUT);
  pinMode(SLP, OUTPUT);
  pinMode(StepPin, OUTPUT);
  pinMode(DirPin, OUTPUT);
  digitalWrite(RST,HIGH);

}



void loop(){

  // go down
//  digitalWrite(En2, HIGH);
//  delayMicroseconds(100000); 
  digitalWrite(En, LOW);
  digitalWrite(SLP, HIGH);
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, LOW);
  digitalWrite(MS3, LOW);
  digitalWrite(DirPin, LOW); 
  digitalWrite(StepPin, HIGH);
  delay(delaytime);
  digitalWrite(StepPin, LOW);
  delay(delaytime);
  delay(35);
}
