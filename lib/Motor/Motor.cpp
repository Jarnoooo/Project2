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
  analogwrite(_leftbackwardPin, speed);
  digitalWrite(_rightForwardPin, LOW);
  analogwrite(_rightBackwardPin, speed);
}
void Motor::stop() {
  digitalWrite(_leftForwardPin, LOW);
  digitalWrite(_leftbackwardPin, LOW);
  digitalWrite(_rightForwardPin, LOW);
  digitalWrite(_rightBackwardPin, LOW);
}
void Motor::turnLeft() {
  analogwrite(_leftForwardPin, speed);
  digitalWrite(_leftbackwardPin, LOW);
  digitalWrite(_rightForwardPin, LOW);
  digitalWrite(_rightBackwardPin, LOW);
}

void Motor::turnRight() {
  digitalWrite(_leftForwardPin, LOW);
  digitalWrite(_leftbackwardPin, LOW);
  analogwrite(_rightForwardPin, speed);
  digitalWrite(_rightBackwardPin, LOW);
}
