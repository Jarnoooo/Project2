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

#define maxSpeed 255

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

void setup() 
{
  Serial.begin(9600);

  motor.speed = maxSpeed; //set motor speed

  pinMode(leftIrSensorPin, INPUT);
  pinMode(midLeftIrSensorPin, INPUT);
  pinMode(midRightIrSensorPin, INPUT);
  pinMode(rightIrSensorPin, INPUT);

  pinMode(sonarTriggerPin, OUTPUT);
  pinMode(sonarEchoPin, INPUT);
}

void loop()
{
  if(isObjectDetected()){ // when 1 is returned from function, stop the robot.
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

  if(lineLost == 1 && (midLeftIrValue || midRightIrValue)) {  // check if line is detected to determine 
    Serial.println("line detected again");
    lineLost = 0;
  }

  if(leftIrValue && midLeftIrValue && midRightIrValue && rightIrValue) { //t-junction
    Serial.println("t junction");
    motor.driveForward();
  }else if(leftIrValue && midLeftIrValue && midRightIrValue) { // turn left
    Serial.println("turn left");
    motor.speed = maxSpeed; // max speed because one motor doesn't have enough torque to move the robot through the bend
    motor.turnLeft();
  }else if(rightIrValue && midRightIrValue && midLeftIrValue) { //turn right
    Serial.println("turn right");
    motor.speed = maxSpeed;
    motor.turnRight();
  }else if(leftIrValue == 0 && midLeftIrValue == 0 && midRightIrValue == 0 && rightIrValue ==0) { // no line is detected move forward.
    Serial.println("no line detected");

    currentMillis = millis(); // initialise timer

    if(lineLost == 0) { // when line is lost, start timer 
      lastLine = millis();
      lineLost = 1;
    }

    if(currentMillis - lastLine > interval && lineLost == 1){ //robot must rotate when the line is still and longer lost then interval
      Serial.println("no line detected -> rotation car");
      motor.turnRight();
    } else {
      motor.driveForward(); // just drive forward
    }
  }else if(midRightIrValue == 0 && rightIrValue == 0) { //right offset
    Serial.println("right offset");
    motor.speed = 230;
    motor.turnLeft();
  }else if(midLeftIrValue == 0 && leftIrValue == 0) { //left offset
    Serial.println("left offset");
    motor.speed = 230;
    motor.turnRight();
  }else if(midLeftIrValue && midRightIrValue) { // forward
    Serial.println("driving forward");
    motor.speed = maxSpeed;
    motor.driveForward();
  }
}

int isObjectDetected()
{   
  digitalWrite(sonarTriggerPin, LOW);
  delayMicroseconds(5); // sensor gives low pulse to clean any high pulse for 5 microseconds
  digitalWrite(sonarTriggerPin, HIGH);
  delayMicroseconds(10); // sensor gives high pulse for 10 microseconds
  digitalWrite(sonarTriggerPin, LOW);
  /* 
  Read the signal from the sensor: a high pulse whose duration is the time (in microseconds) from the sending
  of the ping to the reception of its echo off of an object.
  */
  pinMode(sonarEchoPin, INPUT); 
  /*
  Measures the distance between an object and the robot using PulseIn() to determine
  the amount of time it took for the sound to bounce back from the object.
  */
  long duration = pulseIn(sonarEchoPin, HIGH);
  // Convert the time into a distance
  cm = (duration/2) / 0.0343;     // Duration is in miliseconds, multiply by 0.0343 to get to centimeters
  Serial.println(cm);
  if (cm < 20){  
    return 1; // return 1 to first if function, to stop the robot
  }
  return 0; // return 0 to first if function, that robot can drive on
}
