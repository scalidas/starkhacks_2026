#ifndef MOTOR_H
#define MOTOR_H
#include "utils.h"
#include <Arduino.h> // Needed for float, Serial, etc.

class Motor {
  private:
    MotorPinSet pins;
    static int pwm_channel;
  
  public:
    Motor(MotorPinSet pins_);
    void move(int speed, bool forward);
};

#endif