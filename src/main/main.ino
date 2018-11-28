#include <Arduino.h>
#include <Motor.h>

#define leftForward 2
#define leftBackward 3
#define rightForward 4
#define rightBackward 5
#define leftIrSensor 14 //14 = analog 0 port, 15 = analog 1 port etc.
#define midLeftIrSensor 15
#define midRightIrSensor 16
#define rightIrSensor 17 

Motor motor(leftForward, leftBackward, rightForward, rightBackward);

void setup() {
  Serial.begin(9600);

  /* Configure pins */
  pinMode(14, INPUT);
  pinMode(15, INPUT);
  pinMode(16, INPUT);
  pinMode(17, INPUT);
}

void loop() {
  
  ultrasonic();
  motor.driveForward();
  delay(2000);
  motor.stop();
  delay(2000);
  
  //must be interfaced later    
  int irsensorlinks = analogRead(2);
  int irsensorrechts = analogRead(3);
  int irsensormidden1 = analogRead(4);
  int irsensormidden2 = analogRead(5);

  Serial.println(irsensorlinks);
  Serial.println(irsensorrechts);
  Serial.println(irsensormidden1);
  Serial.println(irsensormidden2);

  if (irsensorlinks == 0 && irsensorrechts == 1) {
    // bocht naar links
    // motor rechts sneller laten draaien
  }
  if (irsensorlinks == 1 && irsensorrechts == 0){
    // bocht naar rechts
    // motor links sneller laten draaien
  }
  if (irsensormidden1 == 1 && irsensormidden2 == 1){
    //doorrijden geen bocht
  }
  if (irsensorlinks == 1 && irsensorrechts == 1 && irsensormidden1 == 1 && irsensormidden2 == 1){
   

}

void ultraSonic () {
  
digitalWrite(echoPin, LOW);
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);

int waardeSonar = pulseIn(echoPin,HIGH);

Serial.println(waardeSonar); 
delay(200);

if (waardeSonar < 1000){
  motor.stop();
  Serial.println("STOPPP!!!");
  
}else{
 motor.driveForward();
  Serial.println("Weer verder rijden");
  }

}
