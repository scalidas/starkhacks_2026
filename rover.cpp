#include <ESP32Servo.h>
#include "rover.h"


Rover::Rover(ServoPins default_servo_pins, ServoPositions default_pos, MotorPins default_motor_pins) : 
servo_fl(), servo_fr(), servo_bl(), servo_br(),
motor_fl(default_motor_pins.fl), motor_fr(default_motor_pins.fr), motor_bl(default_motor_pins.bl), motor_br(default_motor_pins.br)
{
  servo_pins = default_servo_pins;
  motor_pins = default_motor_pins;

  servo_fl.attach(servo_pins.fl);
  servo_fr.attach(servo_pins.fr);
  servo_bl.attach(servo_pins.bl);
  servo_br.attach(servo_pins.br);

  servo_positions = default_pos;
  write_servo_positions();



  set_motor_speed(0, 1);
}

void Rover::set_servo_positions(ServoPositions target_positions) {
  servo_positions = target_positions;
}

void Rover::write_servo_positions() {
  servo_fl.write(servo_positions.fl);
  servo_fr.write(servo_positions.fr);
  servo_bl.write(servo_positions.bl);
  servo_br.write(servo_positions.br);
}

void Rover::set_motor_speed(int speed, bool forward) {
  motor_fl.move(speed, forward);
  motor_fr.move(speed, forward);
  motor_bl.move(speed, forward);
  motor_br.move(speed, forward);
}
