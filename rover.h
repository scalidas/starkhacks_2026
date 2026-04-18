#ifndef ROVER_H
#define ROVER_H

#include <ESP32Servo.h>
#include "utils.h"
#include "motor.h"
#include "pid_controller.h"

class Rover {
  private:
    ServoPins servo_pins;
    ServoPositions servo_positions;
    ServoPositions neutral_positions;

    Servo servo_fl, servo_fr, servo_bl, servo_br;

    MotorPins motor_pins;
    Motor motor_fl, motor_fr, motor_bl, motor_br;

    PIDController pid_controller;

    unsigned long last_update;

  public:
    Rover(ServoPins default_servo_pins, ServoPositions default_pos, MotorPins default_motor_pins);
    
    void set_servo_positions(ServoPositions target_positions);
    void write_servo_positions();

    void set_motor_speed(int speed, bool forward);

    void update();
};


#endif