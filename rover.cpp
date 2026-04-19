#include <ESP32Servo.h>
#include "rover.h"
#include "imu.h"
#include <algorithm>

Rover::Rover(ServoPins default_servo_pins, ServoPositions default_pos, MotorPins default_motor_pins) : 
servo_fl(), servo_fr(), servo_bl(), servo_br(),
motor_fl(default_motor_pins.fl), motor_fr(default_motor_pins.fr), motor_bl(default_motor_pins.bl), motor_br(default_motor_pins.br),
pid_controller({0, 0}, 2.5, 0, 0)
{
  servo_pins = default_servo_pins;
  motor_pins = default_motor_pins;

  servo_fl.attach(servo_pins.fl);
  servo_fr.attach(servo_pins.fr);
  servo_bl.attach(servo_pins.bl);
  servo_br.attach(servo_pins.br);

  servo_positions = default_pos;
  neutral_positions = default_pos;

  write_servo_positions();

  set_motor_speed(0, 1);
}

void Rover::update() {
  //Get IMU data
  Orientation curr_orientation = getFilteredOrientation();

  unsigned long current_time = millis();
  float dt = (float) (current_time - last_update) / 1000;

  AttitudeCorrections corr = pid_controller.compute(curr_orientation, dt);
  last_update = current_time;

  ServoPositions cmd;

  // Mapping (IMPORTANT PART)
  cmd.fl = std::clamp((float)neutral_positions.fl - corr.pitch + corr.roll, 0.0f, 180.0f);
  cmd.fr = std::clamp((float)neutral_positions.fr + corr.pitch - corr.roll, 0.0f, 180.0f);
  cmd.bl = std::clamp((float)neutral_positions.bl + corr.pitch + corr.roll, 0.0f, 180.0f);
  cmd.br = std::clamp((float)neutral_positions.br - corr.pitch - corr.roll, 0.0f, 180.0f);

  set_servo_positions(cmd);
  write_servo_positions();
}

void Rover::set_servo_positions(ServoPositions target_positions) {
  servo_positions = target_positions;
}

void Rover::set_kp(float new_kp) {
  pid_controller.set_kp(new_kp);
}

void Rover::set_kd(float new_kd) {
  pid_controller.set_kd(new_kd);
}

void Rover::set_ki(float new_ki) {
  pid_controller.set_ki(new_ki);
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
