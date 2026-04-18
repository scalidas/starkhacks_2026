#include <Arduino.h>
#include "imu.h"
#include "motor.h"
#include "rover.h"

ServoPins servopins = {4, 5, 6, 7};
ServoPositions servopos = {180, 0, 180, 0};
MotorPins motorpins = {0, 0, 0, 0};

Rover rover(servopins, servopos, motorpins);

String inputBuffer = "";

void setup() {
  Serial.begin(115200);
  initIMU();

  Serial.println("Enter servo positions: fl fr bl br (0-180)");
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    // End of line → parse input
    if (c == '\n') {
      int fl, fr, bl, br;

      // Parse 4 integers
      int parsed = sscanf(inputBuffer.c_str(), "%d %d %d %d", &fl, &fr, &bl, &br);

      if (parsed == 4) {
        // Clamp values (safety)
        servopos.fl = constrain(fl, 0, 180);
        servopos.fr = constrain(fr, 0, 180);
        servopos.bl = constrain(bl, 0, 180);
        servopos.br = constrain(br, 0, 180);

        rover.set_servo_positions(servopos);
        rover.write_servo_positions();

        Serial.print("Updated: ");
        Serial.print(servopos.fl); Serial.print(" ");
        Serial.print(servopos.fr); Serial.print(" ");
        Serial.print(servopos.bl); Serial.print(" ");
        Serial.println(servopos.br);
      } else {
        Serial.println("Invalid format. Use: fl fr bl br");
      }

      inputBuffer = ""; // reset buffer
    } else {
      inputBuffer += c;
    }
  }
}

void loop() {
  handleSerial();

  // Orientation o = getFilteredOrientation();

  // Serial.print("Roll: ");
  // Serial.print(o.roll, 1);
  // Serial.print(" | Pitch: ");
  // Serial.println(o.pitch, 1);

  delay(100);
}