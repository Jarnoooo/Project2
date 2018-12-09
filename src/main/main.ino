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

/*
  Using INPUT_PULLUP the output of the ir modules defaults to 0 if nsso line is
  detected.
*/
int midLeftIrValue = 0;
int rightIrValue = 0;
int midRightIrValue = 0;
int leftIrValue = 0;
int isObjectDetected();

long cm;
Motor motor(leftMotorForwardPin, leftMotorReversePin, rightMotorForwardPin, rightMotorReversePin);

void setup() {
  Serial.begin(9600);
ss
  motor.speed = 150;

  pinMode(leftIrSensorPin, INPUT_PULLUP);
  pinMode(midLeftIrSensorPin, INPUT_PULLUP);
  pinMode(midRightIrSensorPin, INPUT_PULLUP);
  pinMode(rightIrSensorPin, INPUT_PULLUP);

  pinMode(sonarTriggerPin, OUTPUT);
  pinMode(sonarEchoPin, INPUT);
}

void loop() {
  if(isObjectDetected()= 1){
    motor.stop();
    return;
  }
// motor.driveForward();
//  delay(2000);
//  motor.stop();
//  delay(2000);

  //must be interfaced later
  leftIrValue = digitalRead(leftIrSensorPin);
  midLeftIrValue = digitalRead(midLeftIrSensorPin);
  midRightIrValue = digitalRead(midRightIrSensorPin);
  rightIrValue = digitalRead(rightIrSensorPin);

//  Serial.println(leftIrValue);
//  Serial.println(rightIrValue);
//  Serial.println(midRightIrValue);
//  Serial.println(midLeftIrValue);

  // if (leftIrSensor == 0 && rightIrSensor == 1 && midRightIrSensor == 1 && midLeftIrSensor == 1) {
  //   // 90 graden bocht naar links
  // }
  // if (leftIrSensor == 1 && rightIrSensor == 0 && midRightIrSensor == 1 && midLeftIrSensor == 1){
  //   // 90 graden bocht naar rechts
  // }
  // if (midRightIrSensor == 1 && midLeftIrSensor == 1){
  //   // doorrijden geen bocht
  // }
  // if (leftIrSensor == 1 && rightIrSensor == 0 && midRightIrSensor == 1 && midLeftIrSensor == 1){
  //  // flauwe bocht naar links
  // }
  // if (leftIrSensor == 0 && rightIrSensor == 1 && midRightIrSensor == 1 && midLeftIrSensor == 1){
  //   // flauwe bocht naar rechts
  // }
}

/*
  Measures the distance between an object and the robot using PulseIn() to determine
  the amount of time it took for the sound to bounce back from the object.
*/
int isObjectDetected (){ // met int geef je return value aan
 
  // The sensor is triggered by a high pulse of 10 or more microseconds.
  // Give a short low pulse beforehand to ensure a clean high pulse:
  
  digitalWrite(sonarTriggerPin, LOW);
  delayMicroseconds(5);
  digitalWrite(sonarTriggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sonarTriggerPin, LOW);

  // Read the signal from the sensor: a high pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  
  pinMode(sonarEchoPin, INPUT);
  long duration = pulseIn(sonarEchoPin, HIGH);
  
  // Convert the time into a distance by deviding by 2 and devide bu 29.1... you could multiply by 0.0343 instead of deviding by 29.1

  cm = (duration/2) / 29.1;    
  Serial.println(cm);
  Serial.println("cm");

  delay(100);

  if (cm < 20){  // 20 = 20 cm in range
    return 1;
  }

  return 0;
}
