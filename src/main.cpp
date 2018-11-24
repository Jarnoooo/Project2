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

Motor motor(leftForward, leftBackward, rightForward, rightBackward);

void setup() {
  Serial.begin(9600);

  /* Configure pins */
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT);
  pinMode(9, INPUT);
}

void loop() {
  motor.driveForward();
  delay(2000);
  motor.stop();
  delay(2000);

  //must be interfaced later
  int irsensorlinks = digitalRead(2);
  int irsensorrechts = digitalRead(3);
  int irsensormidden1 = digitalRead(4);
  int irsensormidden2 = digitalRead(5);

  Serial.println(irsensorlinks);


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

}
