#ifndef PID_H
#define PID_H
#include "utils.h"

class PIDController {
  private:
    float kp, kd, ki;

    Orientation target;

    // State for roll
    float prev_error_roll;
    float integral_roll;

    // State for pitch
    float prev_error_pitch;
    float integral_pitch;

  public:
    PIDController(Orientation init_target, float init_kp, float init_kd, float init_ki);

    void set_target(Orientation new_target);

    void set_kp(float new_kp);
    void set_kd(float new_kd);
    void set_ki(float new_ki);

    AttitudeCorrections compute(Orientation current, float dt);
};

#endif