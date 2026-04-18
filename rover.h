#ifndef ROVER_H
#define ROVER_H

#include <ESP32Servo.h>

class Rover {
  private:
    int pin_fl, pin_fr, pin_bl, pin_br;
    int pos_fl, pos_fr, pos_bl, pos_br;

    Servo fl, fr, bl, br;

  public:
    Rover(int pinfl, int pinfr, int pinbl, int pinbr);
    
    void update();
}


#endif