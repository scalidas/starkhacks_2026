#include "pid_controller.h"
#include "utils.h"
#include <cmath>
#include <Arduino.h>

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
  if (dt <= 0) return last_out; // Guard against division by zero

  // 1. Errors (No deadband here!)
  float err_roll  = target.roll  - current.roll;
  float err_pitch = target.pitch - current.pitch;

  // 2. Integral with Anti-Windup (Clamping)
  float i_limit = 20.0; // Adjust based on your servo range
  integral_roll  = std::clamp(integral_roll + (err_roll * dt), -i_limit, i_limit);
  integral_pitch = std::clamp(integral_pitch + (err_pitch * dt), -i_limit, i_limit);

  // 3. Derivative on Measurement (Reduces jitter/kicks)
  // We use (current - prev) instead of (error - prev_error) to avoid spikes during target changes
  float d_roll  = (current.roll  - prev_roll)  / dt;
  float d_pitch = (current.pitch - prev_pitch) / dt;

  // 4. PID Output
  out.roll  = (kp * err_roll)  + (ki * integral_roll)  - (kd * d_roll);
  out.pitch = (kp * err_pitch) + (ki * integral_pitch) - (kd * d_pitch);

  // Save state
  prev_roll = current.roll;
  prev_pitch = current.pitch;
  last_out = out;

  return out;
}