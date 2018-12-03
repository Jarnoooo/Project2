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

#define sonarEchoPin 10
#define sonarTriggerPin 11

/*
  Using INPUT_PULLUP the output of the ir modules defaults to 0 if no line is
  detected.
*/
int midLeftIrValue = 0;
int rightIrValue = 0;
int midRightIrValue = 0;
int leftIrValue = 0;

Motor motor(leftMotorForwardPin, leftMotorReversePin, rightMotorForwardPin, rightMotorReversePin);

void setup() {
  Serial.begin(9600);

  pinMode(leftIrSensorPin, INPUT_PULLUP);
  pinMode(midLeftIrSensorPin, INPUT_PULLUP);
  pinMode(midRightIrSensorPin, INPUT_PULLUP);
  pinMode(rightIrSensorPin, INPUT_PULLUP);

}

void loop() {
  int stopCar = isObjectDetected();

// motor.driveForward();
//  delay(2000);
//  motor.stop();
//  delay(2000);

  //must be interfaced later
  leftIrValue = digitalRead(leftIrSensorPin);
  midLeftIrValue = digitalRead(midLeftIrSensorPin);
  midRightIrValue = digitalRead(midRightIrSensorPin);
  rightIrValue = digitalRead(rightIrSensorPin);

  Serial.println(leftIrValue);
  Serial.println(midLeftIrValue);
  Serial.println(midRightIrValue);
  Serial.println(rightIrValue);

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

  digitalWrite(sonarEchoPin, LOW);
  digitalWrite(sonarTriggerPin, HIGH);

  delayMicroseconds(10);

  digitalWrite(sonarTriggerPin, LOW);

  int valueSonar = pulseIn(sonarEchoPin,HIGH);

  Serial.println(valueSonar);
  //delay(200);

  if (valueSonar < 1200){  // 12200 = 10 cm in range
    //Serial.println("object!");
    return 1;
  }
  //Serial.println("no object!");
  return 0;
}
