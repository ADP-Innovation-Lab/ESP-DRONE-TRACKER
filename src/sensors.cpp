#include "sensors.h"

/*-------------------------- IMU Global defs ----------------------------*/
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
Adafruit_9DOF dof = Adafruit_9DOF();

volatile float imu_roll = 0.0, imu_pitch = 0.0, imu_yaw = 0.0;

sensors_event_t accel_event;
sensors_event_t mag_event;
sensors_event_t gyro_event;
sensors_event_t imu_temp_event;
sensors_vec_t orientation;

/*-------------------------- BMP280 Global defs ---------------------------*/

#define BMP_ADD 0x77
// BMP280 Calibration values
uint16_t dig_T1, dig_P1;
int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5;
int16_t dig_P6, dig_P7, dig_P8, dig_P9;

// altitude variable
float AltitudeBarometer, AltitudeBarometerStartUp;
int RateCalibrationNumber;
double pressure_hpa;

/*-------------------------- IMU Functions   ---------------------------*/
boolean imu_setup()
{

    if (!lsm.begin())
    {
        /* There was a problem detecting the LSM303 ... check your connections */
        IMU_DGB.println(F("[IMU]Ooops, no LSM9DS1 detected ...!"));
        return false;
    }

    IMU_DGB.println("[IMU] LSM9DS1 init Successfully.");
    // 1.) Set the accelerometer range
    lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G, lsm.LSM9DS1_ACCELDATARATE_10HZ);
    // lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G, lsm.LSM9DS1_ACCELDATARATE_119HZ);
    // lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G, lsm.LSM9DS1_ACCELDATARATE_476HZ);
    // lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G, lsm.LSM9DS1_ACCELDATARATE_952HZ);

    // 2.) Set the magnetometer sensitivity
    lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
    // lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
    // lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
    // lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

    // 3.) Setup the gyroscope
    lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
    // lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
    // lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);

    return true;
}

void imu_loop()
{
    lsm.read();
    lsm.getEvent(&accel_event, &mag_event, &gyro_event, &imu_temp_event);

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
    unsigned status = Wire.begin();
    if (!status)
    {
        IMU_DGB.println(F("[BMP]Could not find a valid BMP280 sensor, check wiring or "
                          "try a different address!"));
        return false;
    }
    else
        IMU_DGB.print(F("[BMP] I2C bmp init success "));
    // set it in normal Mode
    Wire.beginTransmission(BMP_ADD);
    Wire.write(0xF4);
    Wire.write(0x57);
    Wire.endTransmission();

    // Optimize Sensor reading for indoor navigation - the highest resolution mode
    Wire.beginTransmission(BMP_ADD);
    Wire.write(0xF5);
    Wire.write(0x14);
    Wire.endTransmission();

    // Reading Calibration / Trimming parameters  set by the manufacure
    // we need these data in our calibration / compensation stage
    uint8_t data[24], i = 0;
    Wire.beginTransmission(BMP_ADD);
    Wire.write(0x88);
    Wire.endTransmission();
    Wire.requestFrom(BMP_ADD, 24);
    while (Wire.available())
    {
        data[i] = Wire.read();
        i++;
    }
    dig_T1 = (data[1] << 8) | data[0];
    dig_T2 = (data[3] << 8) | data[2];
    dig_T3 = (data[5] << 8) | data[4];
    dig_P1 = (data[7] << 8) | data[6];
    dig_P2 = (data[9] << 8) | data[8];
    dig_P3 = (data[11] << 8) | data[10];
    dig_P4 = (data[13] << 8) | data[12];
    dig_P5 = (data[15] << 8) | data[14];
    dig_P6 = (data[17] << 8) | data[16];
    dig_P7 = (data[19] << 8) | data[18];
    dig_P8 = (data[21] << 8) | data[20];
    dig_P9 = (data[23] << 8) | data[22];
    delay(250);
    bmp_getStartUpAlt();
    return true;
}

float bmp_getStartUpAlt()
{
    for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++)
    {
        barometer_signals();
        AltitudeBarometerStartUp += AltitudeBarometer;
        delay(1);
    }
    AltitudeBarometerStartUp /= 2000;
    return AltitudeBarometerStartUp;
}

float bmp_getRelativeAltitude()
{
    barometer_signals();
    return AltitudeBarometer -= AltitudeBarometerStartUp;
}

void bmp_print()
{
    IMU_DGB.printf("[BMP] Measured hPa: %0.2f - Abs Altitude: %0.2f  - Act Altitude: %0.2f\r\n",
                   pressure_hpa,
                   bmp_getStartUpAlt(),
                   bmp_getRelativeAltitude());
}

void barometer_signals()
{
    Wire.beginTransmission(BMP_ADD); // BMP280 address
    Wire.write(0xF7);
    Wire.endTransmission();

    Wire.requestFrom(BMP_ADD, 6);
    // reading Pressure data - address : 0XF7, 0XF8, 0XF9
    uint32_t press_msb = Wire.read();
    uint32_t press_lsb = Wire.read();
    uint32_t press_xlsb = Wire.read();
    // reading Temprature data - address : 0XFA, 0XFB, 0XFC
    uint32_t temp_msb = Wire.read();
    uint32_t temp_lsb = Wire.read();
    uint32_t temp_xlsb = Wire.read();

    // Read raw P, T - un compendated and un calibrated data
    unsigned long int adc_P = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >> 4);
    unsigned long int adc_T = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4);

    /* We need to calibrate the sensor based on current temprature
       because it affects the pressure reading .
       |--> calibration code , Datasheet page 46
    */
    // get fined tempraure value
    signed long int var1, var2;
    var1 = ((((adc_T >> 3) - ((signed long int)dig_T1 << 1))) * ((signed long int)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T >> 4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;
    signed long int t_fine = var1 + var2;
    //Serial.printf("T_fine = %ld \n", t_fine);

    // calibrate and compensate pressure
    unsigned long int p;
    var1 = (((signed long int)t_fine) >> 1) - (signed long int)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((signed long int)dig_P6);
    var2 = var2 + ((var1 * ((signed long int)dig_P5)) << 1);
    var2 = (var2 >> 2) + (((signed long int)dig_P4) << 16);
    var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((signed long int)dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((signed long int)dig_P1)) >> 15);

    if (var1 == 0)
    {
        p = 0;
    }
    p = (((unsigned long int)(((signed long int)1048576) - adc_P) - (var2 >> 12))) * 3125;

    if (p < 0x80000000)
    {
        p = (p << 1) / ((unsigned long int)var1);
    }
    else
    {
        p = (p / (unsigned long int)var1) * 2;
    }
    var1 = (((signed long int)dig_P9) * ((signed long int)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((signed long int)(p >> 2)) * ((signed long int)dig_P8)) >> 13;
    p = (unsigned long int)((signed long int)p + ((var1 + var2 + dig_P7) >> 4));

    pressure_hpa = (double)p / 100;
    // Serial.printf("Pressure = %0.2f [hPa] \n", pressure_hpa);

    // get altitude in meter
    AltitudeBarometer = 44330 * (1 - pow(pressure_hpa / 1013.25, 1 / 5.255));
    // Serial.printf("Altitude = %0.2f [m] \n", AltitudeBarometer);
}