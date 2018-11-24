#include <Arduino.h>
#include <Motor.h>

#define leftForward 2
#define leftBackward 3
#define rightForward 4
#define rightBackward 5

Motor motor(leftForward, leftBackward, rightForward, rightBackward);

void setup() {
  Serial.begin(9600);
}

void loop() {
  motor.driveForward();
  delay(2000);
  motor.stop();
  delay(2000);
}
