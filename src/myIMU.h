#ifndef MYIMU_H
#define MYIMU_H
// https://github.com/adafruit/Adafruit_9DOF
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_L3GD20_U.h>


#define IMU_DGB    Serial




boolean imu_setup(); 
void imu_loop();
float imu_getRoll();
float imu_getPitch();
float imu_getYaw();
void imu_print();

#endif