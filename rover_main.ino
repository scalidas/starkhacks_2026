#include <Arduino.h>
#include "imu.h"
#include "motor.h"
#include "rover.h"

#define SERVO_PIN_FL 4 //Yellow wire
#define SERVO_PIN_FR 5
#define SERVO_PIN_BL 6
#define SERVO_PIN_BR 7 //Purple wire

/*
20 and 21 reserved for IMU
*/

//Motor FL and FR are on A pins. Motor BL and BR are on B pins. Left controller and right controller
#define MOTOR_FL_IN1 9
#define MOTOR_FL_IN2 10
#define MOTOR_FL_PWM 11
MotorPinSet pinset_fl = {MOTOR_FL_IN1, MOTOR_FL_IN2, MOTOR_FL_PWM};

#define MOTOR_FR_IN1 17
#define MOTOR_FR_IN2 18
#define MOTOR_FR_PWM 12
MotorPinSet pinset_fr = {MOTOR_FR_IN1, MOTOR_FR_IN2, MOTOR_FR_PWM};


#define MOTOR_BL_IN1 9
#define MOTOR_BL_IN2 10
#define MOTOR_BL_PWM 13
MotorPinSet pinset_bl = {MOTOR_BL_IN1, MOTOR_BL_IN2, MOTOR_BL_PWM};


#define MOTOR_BR_IN1 17
#define MOTOR_BR_IN2 18
#define MOTOR_BR_PWM 14
MotorPinSet pinset_br = {MOTOR_BR_IN1, MOTOR_BR_IN2, MOTOR_BR_PWM};

ServoPins servopins = {SERVO_PIN_FL, SERVO_PIN_FR, SERVO_PIN_BL, SERVO_PIN_BR};
MotorPins motorpins = {pinset_fl, pinset_fr, pinset_bl, pinset_br};

MotorSpeeds motor_speeds = {0, 0, 0, 0, 0, 0, 0, 0};

ServoPositions servopos = {100, 95, 110, 90};

Rover rover(servopins, servopos, motorpins);

String inputBuffer = "";

unsigned long last_time;

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      float p, i, d;
      int fl, fr, bl, br;
      int val, val2, val3;

      // --- PID controls ---
      if (sscanf(inputBuffer.c_str(), "PID %f %f %f", &p, &i, &d) == 3) {
        rover.set_kp(p);
        rover.set_ki(i);
        rover.set_kd(d);

        Serial.print("Set PID → ");
        Serial.print(p); Serial.print(" ");
        Serial.print(i); Serial.print(" ");
        Serial.println(d);
      }
      else if (sscanf(inputBuffer.c_str(), "P %f", &p) == 1) {
        rover.set_kp(p);
        Serial.print("Set Kp → "); Serial.println(p);
      }
      else if (sscanf(inputBuffer.c_str(), "I %f", &i) == 1) {
        rover.set_ki(i);
        Serial.print("Set Ki → "); Serial.println(i);
      }
      else if (sscanf(inputBuffer.c_str(), "D %f", &d) == 1) {
        rover.set_kd(d);
        Serial.print("Set Kd → "); Serial.println(d);
      }

      // --- SERVO: set all ---
      else if (sscanf(inputBuffer.c_str(), "S %d %d %d %d", &fl, &fr, &bl, &br) == 4) {
        servopos.fl = constrain(fl, 0, 180);
        servopos.fr = constrain(fr, 0, 180);
        servopos.bl = constrain(bl, 0, 180);
        servopos.br = constrain(br, 0, 180);

        rover.set_servo_positions(servopos);
        rover.write_servo_positions();

        Serial.print("Servos → ");
        Serial.print(servopos.fl); Serial.print(" ");
        Serial.print(servopos.fr); Serial.print(" ");
        Serial.print(servopos.bl); Serial.print(" ");
        Serial.println(servopos.br);
      }

      // --- SERVO: individual ---
      else if (sscanf(inputBuffer.c_str(), "FL %d", &val) == 1) {
        servopos.fl = constrain(val, 0, 180);
        rover.set_servo_positions(servopos);
        rover.write_servo_positions();
        Serial.print("FL → "); Serial.println(servopos.fl);
      }
      else if (sscanf(inputBuffer.c_str(), "FR %d", &val) == 1) {
        servopos.fr = constrain(val, 0, 180);
        rover.set_servo_positions(servopos);
        rover.write_servo_positions();
        Serial.print("FR → "); Serial.println(servopos.fr);
      }
      else if (sscanf(inputBuffer.c_str(), "BL %d", &val) == 1) {
        servopos.bl = constrain(val, 0, 180);
        rover.set_servo_positions(servopos);
        rover.write_servo_positions();
        Serial.print("BL → "); Serial.println(servopos.bl);
      }
      else if (sscanf(inputBuffer.c_str(), "BR %d", &val) == 1) {
        servopos.br = constrain(val, 0, 180);
        rover.set_servo_positions(servopos);
        rover.write_servo_positions();
        Serial.print("BR → "); Serial.println(servopos.br);
      } 
      else if (sscanf(inputBuffer.c_str(), "M %d %d", &val, &val2) == 2) {
        rover.set_motor_speed(val, val2);
        Serial.print("Motor → "); Serial.println(val);
      }
      else if (sscanf(inputBuffer.c_str(), "M %d %d %d", &val, &val2, &val3) == 2) {
        if (val == 0) {
          motor_speeds.fl_speed = val2;
          motor_speeds.fl_dir = val3;
        } else if (val == 1) {
          motor_speeds.fr_speed = val2;
          motor_speeds.fr_dir = val3;
        } else if (val == 2) {
          motor_speeds.bl_speed = val2;
          motor_speeds.bl_dir = val3;
        } else {
          motor_speeds.br_speed = val2;
          motor_speeds.br_dir = val3;
        }

        rover.set_motor_speed(motor_speeds);
        Serial.print("Motor → "); Serial.println(val2);
      }
      else {
        Serial.println("Invalid command.");
        Serial.println("PID: P x | I x | D x | PID p i d");
        Serial.println("Servo: S fl fr bl br | FL x | FR x | BL x | BR x");
      }

      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  initIMU();

  Serial.println("PID tuning ready:");
  Serial.println("Commands:");
  Serial.println("  P <value>");
  Serial.println("  I <value>");
  Serial.println("  D <value>");
  Serial.println("  PID <p> <i> <d>");

  last_time = 0;
}

void loop() {
  handleSerial();
  unsigned long current_time = millis();
  if (current_time - last_time > 20) {
    last_time = current_time;
    rover.update();
  }
}