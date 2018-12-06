#include <Arduino.h>
#include <Motor.h>

#define leftMotorForwardPin 2
#define leftMotorReversePin 3
#define rightMotorForwardPin 4
#define rightMotorReversePin 5

#define leftIrSensorPin 6
#define midLeftIrSensorPin 7
#define midRightIrSensorPin 8
#define rightIrSensorPin 9

#define sonarTriggerPin 10
#define sonarEchoPin 11

/*
  Using INPUT_PULLUP the output of the ir modules defaults to 0 if no line is
  detected.
*/
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

  pinMode(leftIrSensorPin, INPUT_PULLUP);
  pinMode(midLeftIrSensorPin, INPUT_PULLUP);
  pinMode(midRightIrSensorPin, INPUT_PULLUP);
  pinMode(rightIrSensorPin, INPUT_PULLUP);

  pinMode(sonarTriggerPin, OUTPUT);
  pinMode(sonarEchoPin, INPUT);
}

void loop() {
  if(isObjectDetected()){
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
  Serial.println(cm);
  Serial.println("cm");

  delay(100);

  if (cm < 20){  // 20 = 20 cm in range
    return 1;
  }

  return 0;
}
