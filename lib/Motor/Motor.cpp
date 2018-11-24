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
  digitalWrite(_leftForwardPin, HIGH);
  digitalWrite(_leftbackwardPin, LOW);
  digitalWrite(_rightForwardPin, HIGH);
  digitalWrite(_rightBackwardPin, LOW);
}

void Motor::driveBackward() {
  digitalWrite(_leftForwardPin, LOW);
  digitalWrite(_leftbackwardPin, HIGH);
  digitalWrite(_rightForwardPin, LOW);
  digitalWrite(_rightBackwardPin, HIGH);
}
void Motor::stop() {
  digitalWrite(_leftForwardPin, LOW);
  digitalWrite(_leftbackwardPin, LOW);
  digitalWrite(_rightForwardPin, LOW);
  digitalWrite(_rightBackwardPin, LOW);
}
void Motor::turnLeft() {
  digitalWrite(_leftForwardPin, HIGH);
  digitalWrite(_leftbackwardPin, LOW);
  digitalWrite(_rightForwardPin, LOW);
  digitalWrite(_rightBackwardPin, LOW);
}

void Motor::turnRight() {
  digitalWrite(_leftForwardPin, LOW);
  digitalWrite(_leftbackwardPin, LOW);
  digitalWrite(_rightForwardPin, HIGH);
  digitalWrite(_rightBackwardPin, LOW);
}
