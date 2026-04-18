#ifndef IMU_H
#define IMU_H

#include <Arduino.h> // Needed for float, Serial, etc.
#include <Adafruit_MPU6050.h>
#include "utils.h"

#define I2C_SDA 20
#define I2C_SCL 21

// Function Prototypes
void initIMU();
Orientation getFilteredOrientation();

#endif