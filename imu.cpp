#include "IMU.h"
#include "Wire.h"

// Internal variables (not visible to the .ino file)
Adafruit_MPU6050 mpu;
const float alpha = 0.96;
unsigned long lastTime;
float gyroXerror = 0, gyroYerror = 0;
Orientation currentOrientation = {0.0, 0.0};

void initIMU() {
  Wire.begin(I2C_SDA, I2C_SCL);

  if (!mpu.begin()) {
    while (1) yield(); 
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  
  // Quick internal calibration
  for (int i = 0; i < 200; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gyroXerror += g.gyro.x;
    gyroYerror += g.gyro.y;
    delay(2);
  }
  gyroXerror /= 200;
  gyroYerror /= 200;
  lastTime = millis();
}

Orientation getFilteredOrientation() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float dt = (millis() - lastTime) / 1000.0;
  lastTime = millis();

  float accelRoll = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  float accelPitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;

  float gyroXrate = (g.gyro.x - gyroXerror) * 180 / PI;
  float gyroYrate = (g.gyro.y - gyroYerror) * 180 / PI;

  currentOrientation.roll = alpha * (currentOrientation.roll + gyroXrate * dt) + (1 - alpha) * accelRoll;
  currentOrientation.pitch = alpha * (currentOrientation.pitch + gyroYrate * dt) + (1 - alpha) * accelPitch;

  return currentOrientation;
}