#include <Arduino.h> // Needed for float, Serial, etc.
#include "imu.h"
#include "rover.h"
#include "motor.h"

Motor motor1({16, 17, 18});
Motor motor2({11, 12, 13});



void setup() {
  Serial.begin(115200);

}

void loop() {
  motor1.move(100, 1);
  motor2.move(100, 1);
  delay(3000);
  motor1.move(255, 1);
  motor2.move(255, 1);
  delay(3000);
  motor1.move(0, 1);
  motor2.move(0, 1);
  delay(3000);
}