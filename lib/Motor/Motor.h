#ifndef Motor_H
#define Motor_H

#include <Arduino.h>

class Motor {
  private:
    int _leftForwardPin;
    int _leftbackwardPin;
    int _rightForwardPin;
    int _rightBackwardPin;
  public:
    Motor(int, int, int, int);
    void driveForward();
    void driveBackward();
    void stop();
    void turnLeft();
    void turnRight();
};

#endif
