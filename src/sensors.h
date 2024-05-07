#ifndef SENSORS_H
#define SENSORS_H

// https://github.com/adafruit/Adafruit_9DOF
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_BMP280.h>

#define IMU_DGB    Serial



//----------------- IMU Functions
boolean imu_setup(); 
void imu_loop();
float imu_getRoll();
float imu_getPitch();
float imu_getYaw();
void imu_print();


//----------------- BMP280 Functions
boolean bmp_setup(); 
void bmp_loop(); 
float bmp_getPressure_hPa(); 
float bmp_abs_altitude();
float bmp_act_altitude(); 
void bmp_print(); 

#endif