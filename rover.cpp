#include <ESP32Servo.h>
#include "rover.h"
#include "imu.h"
#include <algorithm>

Rover::Rover(ServoPins default_servo_pins, ServoPositions default_pos, MotorPins default_motor_pins) : 
servo_fl(), servo_fr(), servo_bl(), servo_br(),
motor_fl(default_motor_pins.fl), motor_fr(default_motor_pins.fr), motor_bl(default_motor_pins.bl), motor_br(default_motor_pins.br),
pid_controller({0, 0}, 0, 0, 0)
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

// Adjust this constant to change how aggressively the robot reacts to tilt
const float SENSITIVITY = 5.5; 
const float WHEELBASE = 169.46;
void Rover::update() {
    // 1. Get current orientation from IMU
    Orientation current = getFilteredOrientation();

    //Calculate the linear distance required for each wheel to travel
    float pitch_correction_mm = WHEELBASE * std::sin(current.pitch * M_PI / 180.0);
    

    // 2. Calculate the required offsets for each wheel
    // We want to move the servo in the OPPOSITE direction of the tilt
    // If pitch is positive (front up), we subtract from front servos to lower the body
    ServoPositions offsets;

    // Pitch compensation: Front and Back move oppositely
    float pitch_adj = current.pitch * SENSITIVITY;
    // Roll compensation: Left and Right move oppositely
    float roll_adj = current.roll * SENSITIVITY;

    // Apply the mix (signs may need flipping depending on your servo orientation)
    offsets.fl = -pitch_adj + roll_adj;
    offsets.fr = pitch_adj - roll_adj;
    offsets.bl =  pitch_adj + roll_adj;
    offsets.br =  -pitch_adj - roll_adj;

    // 3. Apply offsets to neutral positions
    ServoPositions target;
    target.fl = constrain(neutral_positions.fl + offsets.fl, 0, 180);
    target.fr = constrain(neutral_positions.fr + offsets.fr, 0, 180);
    target.bl = constrain(neutral_positions.bl + offsets.bl, 0, 180);
    target.br = constrain(neutral_positions.br + offsets.br, 0, 180);

    // 4. Update the robot hardware
    set_servo_positions(target);
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

void Rover::set_motor_speed(MotorSpeeds speeds) {
  motor_fl.move(speeds.fl_speed, speeds.fl_dir);
  motor_fr.move(speeds.fr_speed, speeds.fr_dir);
  motor_bl.move(speeds.bl_speed, speeds.bl_dir);
  motor_br.move(speeds.br_speed, speeds.br_dir);
}
