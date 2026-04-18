#ifndef UTILS_H
#define UTILS_H

//Orientation. Always in degrees
struct Orientation {
  float roll;
  float pitch;
};


//Struct of servo pins.
struct ServoPins {
  int fl;
  int fr;
  int bl;
  int br;
};

//Struct of servo positions. Always in degrees
struct ServoPositions {
  int fl;
  int fr;
  int bl;
  int br;
};


//Motor pins for one motor
struct MotorPinSet {
  int in1;
  int in2;
  int pwm;
};

//All motor pins
struct MotorPins {
  MotorPinSet fl;
  MotorPinSet fr;
  MotorPinSet bl;
  MotorPinSet br;
};

struct AttitudeCorrections {
  float roll;
  float pitch;
};

// PWM Settings
const int pwm_freq = 5000;
const int pwm_resolution = 8;

#endif