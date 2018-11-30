#include <Arduino.h>
#include <Motor.h>

#define leftForward 2
#define leftBackward 3
#define rightForward 4
#define rightBackward 5
#define leftIrSensor  A0
#define midLeftIrSensor A1
#define midRightIrSensor A2
#define rightIrSensor A3
#define echoPin 8
#define trigPin 9

//Motor motor(leftForward, leftBackward, rightForward, rightBackward);

void setup() {
  Serial.begin(9600);

}

void loop() {
  
  int stopCar = objectDetected ();
  
 motor.driveForward();
  delay(2000);
  motor.stop();
  delay(2000);
  
  //must be interfaced later    
  int leftIrSensor = digitalRead(A0);
  int rightIrSensor = digitalRead(A1);
  int midRightIrSensor = digitalRead(A2);
  int midLeftIrSensor = digitalRead(A3);

  Serial.println(leftIrSensor);
  Serial.println(rightIrSensor);
  Serial.println(midRightIrSensor);
  Serial.println(midLeftIrSensor);

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
  
