#include <Arduino.h>
#include "imu.h"
#include "motor.h"
#include "rover.h"

#define SERVO_PIN_FL 4
#define SERVO_PIN_FR 5
#define SERVO_PIN_BL 6
#define SERVO_PIN_BR 7

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

ServoPositions servopos = {160, 55, 120, 35};

Rover rover(servopins, servopos, motorpins);

String inputBuffer = "";

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      float p, i, d;

      // --- Case 1: full PID update ---
      if (sscanf(inputBuffer.c_str(), "PID %f %f %f", &p, &i, &d) == 3) {
        rover.set_kp(p);
        rover.set_ki(i);
        rover.set_kd(d);

        Serial.print("Set PID → ");
        Serial.print(p); Serial.print(" ");
        Serial.print(i); Serial.print(" ");
        Serial.println(d);
      }

      // --- Case 2: individual updates ---
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
      else {
        Serial.println("Invalid command. Use: P x | I x | D x | PID p i d");
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
}

void loop() {
  handleSerial();

  rover.update();
  delay(50);
}