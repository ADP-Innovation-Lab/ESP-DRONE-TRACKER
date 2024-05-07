#include "sensors.h"

/*-------------------------- IMU Global defs ----------------------------*/
Adafruit_9DOF dof = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(30302);

volatile float imu_roll = 0.0, imu_pitch = 0.0, imu_yaw = 0.0;

sensors_event_t accel_event;
sensors_event_t mag_event;
sensors_vec_t orientation;

/*-------------------------- BMP280 Global defs ---------------------------*/
Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

// got from >> https://www.avmet.ae/omaa.aspx
const float AD_METAR_hPa = 1008.0, ELV_ALTITUDE = 27.0, SEA_LEVEL_hPa =1013.25;
volatile float measure_hPa = 0,actual_hPa= 0, abs_altitude = 0, act_altitude = 0, pressure_dif = 0;

/*-------------------------- IMU Functions   ---------------------------*/
boolean imu_setup()
{

    if (!accel.begin())
    {
        /* There was a problem detecting the LSM303 ... check your connections */
        IMU_DGB.println(F("[IMU]Ooops, no LSM303 detected ... Check your wiring!"));
        return false;
    }
    if (!mag.begin())
    {
        /* There was a problem detecting the LSM303 ... check your connections */
        IMU_DGB.println("[IMU] Ooops, no LSM303 detected ... Check your wiring!");
        return false;
    }
    IMU_DGB.println("[IMU] LSM303 init Successfully.");
    return true;
}


void imu_loop()
{

    /* Read the accelerometer and magnetometer */
    accel.getEvent(&accel_event);
    mag.getEvent(&mag_event);

    /* Use the new fusionGetOrientation function to merge accel/mag data */
    if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation))
    {
        imu_roll = orientation.roll;
        imu_pitch = orientation.pitch;
        imu_yaw = orientation.heading;
    }
    else
        IMU_DGB.println("[IMU] Get orination failed !");
}

float imu_getRoll()
{
    return imu_roll;
}
float imu_getPitch()
{
    return imu_pitch;
}
float imu_getYaw()
{
    return imu_yaw;
}

void imu_print()
{

    IMU_DGB.printf("[IMU] Roll: %0.2f Pitch: %0.2f Yaw: %0.2f \r\n",
                   imu_roll,
                   imu_pitch,
                   imu_yaw);
}

/*-------------------------- BMP280 Functions ---------------------------*/
boolean bmp_setup()
{
    // init bmp I2C, bmp280
    unsigned status = bmp.begin();
    if (!status)
    {
        IMU_DGB.println(F("[BMP]Could not find a valid BMP280 sensor, check wiring or "
                          "try a different address!"));
        // while (1)
        // delay(10);
        return false;
    }
    else
        IMU_DGB.print(F("[BMP] I2C bmp init success "));
    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    /* Calibrate pressure sensor */
    measure_hPa = bmp.readPressure() / 100;
    pressure_dif = AD_METAR_hPa - measure_hPa;
    IMU_DGB.print(F("[BMP] Pressure ERORR = "));
    IMU_DGB.println(pressure_dif);

    return true;
}

void bmp_loop()
{
    measure_hPa = bmp.readPressure() / 100;
    actual_hPa  = measure_hPa + pressure_dif; 

    abs_altitude = bmp.readAltitude(SEA_LEVEL_hPa); 
    act_altitude = (44330 * (1.0 - pow(measure_hPa / SEA_LEVEL_hPa, 0.1903)))-ELV_ALTITUDE;
    
}


float bmp_getPressure_hPa(){
    return actual_hPa; 
}
float bmp_abs_altitude(){
    return abs_altitude; 
}
float bmp_act_altitude(){
    return act_altitude; 
} 


void bmp_print(){
    IMU_DGB.printf("[BMP] Measured hPa: %0.2f - Abs Altitude: %0.2f  - Act Altitude: %0.2f\r\n",
                   measure_hPa,
                   
                   abs_altitude,
                   act_altitude);
}