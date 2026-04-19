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
  float fl;
  float fr;
  float bl;
  float br;
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

//Speeds and directions of motors
struct MotorSpeeds {
  int fl_speed;
  int fl_dir;
  int fr_speed;
  int fr_dir;
  int bl_speed;
  int bl_dir;
  int br_speed;
  int br_dir;
};

// PWM Settings
const int pwm_freq = 5000;
const int pwm_resolution = 8;

#endif