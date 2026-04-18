#include "pid_controller.h"
#include "utils.h"
#include <cmath>

PIDController::PIDController(Orientation init_target, float init_kp, float init_kd, float init_ki) {
  target = init_target;
  kp = init_kp;
  kd = init_kd;
  ki = init_ki;
}

void PIDController::set_target(Orientation new_target) {
  target = new_target;
}

void PIDController::set_kp(float new_kp) {
  kp = new_kp;
}

void PIDController::set_kd(float new_kd) {
  kd = new_kd;
}

void PIDController::set_ki(float new_ki) {
  ki = new_ki;
}

AttitudeCorrections PIDController::compute(Orientation current, float dt) {
  AttitudeCorrections out;

  // Errors. Deadband introduced to avoid overcorrections
  float err_roll  = target.roll  - current.roll;
  if (std::abs(err_roll) < 1.5) err_roll = 0;

  float err_pitch = target.pitch - current.pitch;
  if (std::abs(err_pitch) < 1.5) err_pitch = 0;

  // Integral
  integral_roll  += err_roll * dt;
  integral_pitch += err_pitch * dt;

  // Derivative
  float d_roll  = (err_roll  - prev_error_roll)  / dt;
  float d_pitch = (err_pitch - prev_error_pitch) / dt;

  // PID output
  out.roll  = kp * err_roll  + ki * integral_roll  + kd * d_roll;
  out.pitch = kp * err_pitch + ki * integral_pitch + kd * d_pitch;

  // Save state
  prev_error_roll  = err_roll;
  prev_error_pitch = err_pitch;

  return out;
}