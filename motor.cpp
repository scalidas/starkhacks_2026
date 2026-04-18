#include "motor.h"
#include <Arduino.h> // Needed for float, Serial, etc.

int Motor::pwm_channel = 0;

Motor::Motor(MotorPinSet pins_) {
  pins = pins_;

  pinMode(pins.in1, OUTPUT);
  pinMode(pins.in2, OUTPUT);
  
  // Setup PWM on the PWMA pin
  ledcAttachChannel(pins.pwm, pwm_freq, pwm_resolution, pwm_channel);
  pwm_channel++;

  move(0, 1);
}

//Move the motor in specified speed (0-255) and direction
void Motor::move(int speed, bool forward) {
  // 1. Set Direction
  if (forward) {
    digitalWrite(pins.in1, HIGH);
    digitalWrite(pins.in2, LOW);
  } else {
    digitalWrite(pins.in1, LOW);
    digitalWrite(pins.in2, HIGH);
  }

  // 2. Set Speed
  ledcWrite(pins.pwm, speed);
}

