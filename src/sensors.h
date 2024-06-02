#ifndef SENSORS_H
#define SENSORS_H

// https://github.com/adafruit/Adafruit_9DOF
#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <Adafruit_9DOF.h>


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

float bmp_getStartUpAlt(); 
float bmp_getRelativeAltitude();
void bmp_print(); 
void barometer_signals();
#endif