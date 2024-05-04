#include "myIMU.h"

/* Assign a unique ID to the sensors */
Adafruit_9DOF dof = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(30302);

volatile float imu_roll = 0.0, imu_pitch = 0.0, imu_yaw = 0.0;

sensors_event_t accel_event;
sensors_event_t mag_event;
sensors_vec_t orientation;

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
    IMU_DGB.println("[IMU] LSM303 Setuped Successfully.");
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

float imu_getRoll(){
    return imu_roll ; 
}
float imu_getPitch(){
    return imu_pitch ; 
}
float imu_getYaw(){
    return imu_yaw ; 
}

void imu_print(){

    IMU_DGB.printf("[IMU] Roll: %0.2f Pitch: %0.2f Yaw: %0.2f \r\n",
                        imu_roll,
                        imu_pitch,
                        imu_yaw);

}