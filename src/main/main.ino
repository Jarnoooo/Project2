#include <Arduino.h>
#include <Motor.h>

#define leftMotorForwardPin 3
#define leftMotorReversePin 5
#define rightMotorForwardPin 6
#define rightMotorReversePin 9

#define leftIrSensorPin 2
#define midLeftIrSensorPin 4
#define midRightIrSensorPin 7
#define rightIrSensorPin 8

#define sonarTriggerPin 10
#define sonarEchoPin 11


int midLeftIrValue = 0;
int rightIrValue = 0;
int midRightIrValue = 0;
int leftIrValue = 0;
long cm;

int isObjectDetected();

Motor motor(leftMotorForwardPin, leftMotorReversePin, rightMotorForwardPin, rightMotorReversePin);

void setup() {
  Serial.begin(9600);

  motor.speed = 150;

  pinMode(leftIrSensorPin, INPUT);
  pinMode(midLeftIrSensorPin, INPUT);
  pinMode(midRightIrSensorPin, INPUT);
  pinMode(rightIrSensorPin, INPUT);

  pinMode(sonarTriggerPin, OUTPUT);
  pinMode(sonarEchoPin, INPUT);
}

void loop() {
  // if(isObjectDetected()){
  //   motor.stop();
  //   return;
  // }
// motor.driveForward();
//  delay(2000);
//  motor.stop();
//  delay(2000);

  //must be interfaced later
  leftIrValue = digitalRead(leftIrSensorPin);
  midLeftIrValue = digitalRead(midLeftIrSensorPin);
  midRightIrValue = digitalRead(midRightIrSensorPin);
  rightIrValue = 0; //digitalRead(rightIrSensorPin);

  // Serial.print("leftIrValue: ");
  // Serial.print(leftIrValue);
  // Serial.println();
  // Serial.print("midLeftIrValue: ");
  // Serial.print(midLeftIrValue);
  // Serial.println();
  // Serial.print("midRightIrValue: ");
  // Serial.print(midRightIrValue);
  // Serial.println();

  // 1 -> black line
  // 0 -> white

 if(leftIrValue && midLeftIrValue && midRightIrValue && rightIrValue) { //t-junction
   Serial.println("t junction");

   //move forward
 }else if(leftIrValue && midLeftIrValue) { // turn left
   Serial.println("turn left");

 }else if(rightIrValue && midRightIrValue) { //turn right
   Serial.println("turn right");

 }else if(midRightIrValue == 0 && leftIrValue == 0 && rightIrValue == 0) { //right offset
   Serial.println("right offset");

   //right adjustion
 }else if(midLeftIrValue == 0 && leftIrValue == 0 && rightIrValue == 0) { //left offset
   Serial.println("left offset");
   //left adjustion
 }else if(leftIrValue == 0 && midLeftIrValue == 0 && midRightIrValue == 0 && rightIrValue ==0) { // no line is detected move forward.
   Serial.println("no line detected");
 }else if(midLeftIrValue && midRightIrValue) {
   Serial.println("driving forward");
   motor.speed = 255;
   motor.driveForward();
 }
}

/*
  Measures the distance between an object and the robot using PulseIn() to determine
  the amount of time it took for the sound to bounce back from the object.
*/
int isObjectDetected (){ // met int geef je return value aan
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(sonarTriggerPin, LOW);
  delayMicroseconds(5);
  digitalWrite(sonarTriggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sonarTriggerPin, LOW);

  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(sonarEchoPin, INPUT);
  long duration = pulseIn(sonarEchoPin, HIGH);
  // Convert the time into a distance
  cm = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
  // Serial.println(cm);
  // Serial.println("cm");

  delay(100);

  if (cm < 20){  // 20 = 20 cm in range
    return 1;
  }

  return 0;
}
