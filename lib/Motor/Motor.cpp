#include "Motor.h"

Motor::Motor(int leftForwardPin, int leftbackwardPin, int rightForwardPin, int rightBackwardPin) {
  _leftForwardPin = leftForwardPin;
  _leftbackwardPin = leftbackwardPin;
  _rightForwardPin = rightForwardPin;
  _rightBackwardPin = rightBackwardPin;

  pinMode(leftForwardPin, OUTPUT);
  pinMode(leftbackwardPin, OUTPUT);
  pinMode(rightForwardPin, OUTPUT);
  pinMode(rightBackwardPin, OUTPUT);

}

void Motor::driveForward() {
  analogWrite(_leftForwardPin, speed);
  digitalWrite(_leftbackwardPin, LOW);
  analogWrite(_rightForwardPin, speed);
  digitalWrite(_rightBackwardPin, LOW);
}

void Motor::driveBackward() {
  digitalWrite(_leftForwardPin, LOW);
  analogWrite(_leftbackwardPin, speed);
  digitalWrite(_rightForwardPin, LOW);
  analogWrite(_rightBackwardPin, speed);
}
void Motor::stop() {
  digitalWrite(_leftForwardPin, LOW);
  digitalWrite(_leftbackwardPin, LOW);
  digitalWrite(_rightForwardPin, LOW);
  digitalWrite(_rightBackwardPin, LOW);
}
void Motor::turnLeft() {
  analogWrite(_leftForwardPin, speed);
  digitalWrite(_leftbackwardPin, LOW);
  digitalWrite(_rightForwardPin, LOW);
  analogWrite(_rightBackwardPin, speed);  //full traction on reverse rotation wheel will shake the car off track.
}

void Motor::turnRight() {
  digitalWrite(_leftForwardPin, LOW);
  analogWrite(_leftbackwardPin, speed);
  analogWrite(_rightForwardPin, speed);
  digitalWrite(_rightBackwardPin, LOW);
}
