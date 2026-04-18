#ifndef PID_H
#define PID_H
#include "utils.h"

class PIDController {
  private:
    float kp, kd, ki;
    
    float target_pitch, target_roll;

  public:
    ServoPositions compute_target_positions(float roll, float pitch);
};

#endif