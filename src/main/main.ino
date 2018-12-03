#include <Arduino.h>
//#include <Motor.h>

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
  
// motor.driveForward();
//  delay(2000);
//  motor.stop();
//  delay(2000);
  
  //must be interfaced later    
  leftIrValue = digitalRead(leftIrSensor);
  rightIrValue = digitalRead(rightIrSensor);
  midRightIrValue = digitalRead(midRightIrSensor);
  midLeftIrValue = digitalRead(midLeftIrSensor);

  Serial.println(leftIrValue);
  Serial.println(rightIrValue);
  Serial.println(midRightIrValue);
  Serial.println(midLeftIrValue);

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

  Serial.println(valueSonar); 
//  delay(200);

   if (valueSonar < 1200){  // 12200 = 10 cm in range
//     Serial.println("object!");
      return 1;
    }
//    Serial.println("no object!");
    return 0;
   }
  
