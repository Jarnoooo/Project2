#include <Arduino.h>
#include <Motor.h>

#define leftMotorForwardPin 6
#define leftMotorReversePin 9
#define rightMotorForwardPin 3
#define rightMotorReversePin 5

#define leftIrSensorPin 2
#define midLeftIrSensorPin 4
#define midRightIrSensorPin 7
#define rightIrSensorPin 8

#define sonarTriggerPin 10
#define sonarEchoPin 11

#define maxSpeed 255;

int midLeftIrValue = 0;
int rightIrValue = 0;
int midRightIrValue = 0;
int leftIrValue = 0;

int isObjectDetected();

unsigned long lastLine = 0;
unsigned long currentMillis = 0;
int interval = 500;
int lineLost = 0;

long cm;

Motor motor(leftMotorForwardPin, leftMotorReversePin, rightMotorForwardPin, rightMotorReversePin);

void setup() {
  Serial.begin(9600);

  motor.speed = maxSpeed;

  pinMode(leftIrSensorPin, INPUT);
  pinMode(midLeftIrSensorPin, INPUT);
  pinMode(midRightIrSensorPin, INPUT);
  pinMode(rightIrSensorPin, INPUT);

  pinMode(sonarTriggerPin, OUTPUT);
  pinMode(sonarEchoPin, INPUT);
}

void loop() {
  if(isObjectDetected()){
    motor.stop();
    return;
  }

  //must be interfaced later
  leftIrValue = digitalRead(leftIrSensorPin);
  midLeftIrValue = digitalRead(midLeftIrSensorPin);
  midRightIrValue = digitalRead(midRightIrSensorPin);
  rightIrValue = digitalRead(rightIrSensorPin);

  // 0 -> white
  // 1 -> black

  if(lineLost == 1 && (midLeftIrValue || midRightIrValue)) {
    Serial.println("line detected again");
    lineLost = 0;
  }

  if(leftIrValue && midLeftIrValue && midRightIrValue && rightIrValue) { //t-junction
    Serial.println("t junction");
    motor.driveForward();
  }else if(leftIrValue && midLeftIrValue && midRightIrValue) { // turn left
    Serial.println("turn left");
    motor.speed = maxSpeed;
    motor.turnLeft();
  }else if(rightIrValue && midRightIrValue && midLeftIrValue) { //turn right
    Serial.println("turn right");
    motor.speed = maxSpeed;
    motor.turnRight();
  }else if(leftIrValue == 0 && midLeftIrValue == 0 && midRightIrValue == 0 && rightIrValue ==0) { // no line is detected move forward.
    Serial.println("no line detected");

    currentMillis = millis();

    if(lineLost == 0) {
      lastLine = millis();
      lineLost = 1;
    }

    if(currentMillis - lastLine > interval && lineLost == 1){ //robot must rotate
      Serial.println("no line detected -> rotation car");
      motor.turnRight();
    } else {
      motor.driveForward();
    }
  }else if(midRightIrValue == 0 && rightIrValue == 0) { //right offset
    Serial.println("right offset");
    motor.speed = 230;
    motor.turnLeft();
  }else if(midLeftIrValue == 0 && leftIrValue == 0) { //left offset
    Serial.println("left offset");
    motor.speed = 230;
    motor.turnRight();
  }else if(midLeftIrValue && midRightIrValue) {
    Serial.println("driving forward");
    motor.speed = maxSpeed;
    motor.driveForward();
  }
}

/*
  Measures the distance between an object and the robot using PulseIn() to determine
  the amount of time it took for the sound to bounce back from the object.
*/
int isObjectDetected(){
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
  // Convert the time into a distance
  cm = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
  Serial.println(cm);
  if (cm < 20){  // 20 = 20 cm in range
    return 1;
  }
  return 0;
}
