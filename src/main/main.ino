#include <Arduino.h>
#include <Motor.h>

#define leftForward 2
#define leftBackward 3
#define rightForward 4
#define rightBackward 5

#define leftIrSensor 6
#define midLeftIrSensor 7
#define midRightIrSensor 8 
#define rightIrSensor 9

#define echoPin 10
#define trigPin 11

int midLeftIrValue = 1; // predefining variables
int rightIrValue = 1;
int midRightIrValue = 1;
int leftIrValue = 1;

//Motor motor(leftForward, leftBackward, rightForward, rightBackward);

void setup() {
  
  Serial.begin(9600);
  
  pinMode (6, INPUT);
  pinMode (7, INPUT);
  pinMode (8, INPUT);
  pinMode (9, INPUT);
  
}

void loop() {
  
  int stopCar = objectDetected ();
  
  if (stopCar == 1)
  {
    motor.stop();
    return;
  }
// motor.driveForward();
//  delay(2000);
//  motor.stop();
//  delay(2000);
  
  //must be interfaced later    
  leftIrValue = digitalRead(leftIrSensor);
  rightIrValue = digitalRead(rightIrSensor);
  midRightIrValue = digitalRead(midRightIrSensor);
  midLeftIrValue = digitalRead(midLeftIrSensor);

//  Serial.println(leftIrValue);
//  Serial.println(rightIrValue);
//  Serial.println(midRightIrValue);
//  Serial.println(midLeftIrValue);

  if (leftIrSensor == 0 && rightIrSensor == 1 && midRightIrSensor == 1 && midLeftIrSensor == 1) {
    // 90 graden bocht naar links
  }
  if (leftIrSensor == 1 && rightIrSensor == 0 && midRightIrSensor == 1 && midLeftIrSensor == 1){
    // 90 graden bocht naar rechts
  }
  if (midRightIrSensor == 1 && midLeftIrSensor == 1){
    // doorrijden geen bocht
  }
  if (leftIrSensor == 1 && rightIrSensor == 0 && midRightIrSensor == 1 && midLeftIrSensor == 1){
   // flauwe bocht naar links
  }
  if (leftIrSensor == 0 && rightIrSensor == 1 && midRightIrSensor == 1 && midLeftIrSensor == 1){
    // flauwe bocht naar rechts
  }
}
int objectDetected (){ // met int geef je return value aan

  digitalWrite(echoPin, LOW);
  digitalWrite(trigPin, HIGH);

  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  int valueSonar = pulseIn(echoPin,HIGH);
  
  valueSonar = valueSonar /2 /29 ;  // :2 because of traveltime back and forth ... :29 because speed of sound =343 m/s = 0.0343 cm/ uS = 1/29cm/uS
  
  Serial.println(valueSonar); 
   

   if (valueSonar < 20){  // 20 = 20 cm in range
//     Serial.println("object!");
      return 1;
    }
//    Serial.println("no object!");
    return 0;
   }
  
