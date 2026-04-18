#include "IMU.h"

// Direction Pins
const int ain1 = 16; 
const int ain2 = 17;
// Speed Pin
const int pwma = 18; 

// PWM Settings
const int freq = 5000;
const int channel = 0;
const int resolution = 8;

void moveMotor(int speed, bool forward);

void setup() {
  Serial.begin(115200);
  initIMU();

  pinMode(ain1, OUTPUT);
  pinMode(ain2, OUTPUT);
  
  // Setup PWM on the PWMA pin
  ledcAttachChannel(pwma, freq, resolution, channel);
}

void loop() {

  Orientation data = getFilteredOrientation();
  
  Serial.print("P: "); Serial.print(data.pitch);
  Serial.print(" R: "); Serial.println(data.roll);

  // Forward at 50% speed
  moveMotor(128, true); 
  delay(2000);

  // Stop
  moveMotor(0, true);
  delay(1000);

  // Backward at 75% speed
  moveMotor(192, false);
  delay(2000);
  
}

/**
 * @param speed 0 to 255
 * @param forward true for forward, false for reverse
 */
void moveMotor(int speed, bool forward) {
  // 1. Set Direction
  if (forward) {
    digitalWrite(ain1, HIGH);
    digitalWrite(ain2, LOW);
  } else {
    digitalWrite(ain1, LOW);
    digitalWrite(ain2, HIGH);
  }

  // 2. Set Speed
  ledcWrite(pwma, speed);
}