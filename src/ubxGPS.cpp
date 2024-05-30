/*
  my_gps.cpp - Library for interfacing with UBX GPS module.
            supports both ESP32 and AVR boards
  Created by [Mohamed Maher ]
  Dependency:
        http://librarymanager/All#SparkFun_u-blox_GNSS
*/
#include "ubxGPS.h"

#ifdef __AVR__
#include <SoftwareSerial.h>
SoftwareSerial gpsSerial(gpsRX, gpsTX);
#else
#include <HardwareSerial.h>
HardwareSerial gpsSerial(gpsPORT);
#endif

#define validCheckTime 5000
SFE_UBLOX_GNSS myGNSS;
boolean isValid = false;
boolean newNavData = false;

long lastCheckMills = 0;

void myGPS::setup()
{
#ifdef __AVR__
    gpsSerial.begin(GPSBaud);
#else
    gpsSerial.begin(GPSBaud, SERIAL_8N1, gpsRX, gpsTX);
#endif

    if (myGNSS.begin(gpsSerial) == true)
        GPS_DBG.println(F("[GPS] Serial port init success."));
    else
        GPS_DBG.println(F("[GPS] Error: Serial port init Failed."));
    //myGNSS.enableDebugging();
    myGNSS.setUART1Output(COM_TYPE_UBX); // Set the UART port to output NMEA only
    // myGNSS.setI2COutput(COM_TYPE_UBX);   // Set the I2C port to output UBX only (turn off NMEA noise)
    //myGNSS.setNavigationFrequency(2); // Produce two solutions per second
    //myGNSS.setAutoPVT(true);          // Tell the GNSS to "send" each solution
    // myGNSS.saveConfiguration(); //Optional: Save the current settings to flash and BBR
}

void myGPS::loop()
{

    if (millis() - lastCheckMills > validCheckTime)
    {
        if (myGNSS.getFixType() > 1)
        {
            isValid = true;
            if (myGNSS.getPVT() && (myGNSS.getInvalidLlh() == false))
                newNavData = true;
            else
                newNavData = false;
        }
        else
        {
            isValid = false;
            GPS_DBG.printf("[GPS] Error: No FIX , [%d] \r\n", myGNSS.getFixType());
        }

        lastCheckMills = millis();
    }
}

String myGPS::getDateTimeStr()
{
    // expected : "2024-05-04T14:43:22Z"
    char dt[25];
    // Format the date and time components with leading zeros
    if (myGNSS.getYear() < 2024)
    {
        sprintf(dt, "%04d-%02d-%02dT%02d:%02d:%02dZ",
                2024,                // Year
                01,                  // Month
                01,                  // Day
                myGNSS.getHour(),    // Hour
                myGNSS.getMinute(),  // Minute
                myGNSS.getSecond()); // Second
    }
    else
    {
        sprintf(dt, "%04d-%02d-%02dT%02d:%02d:%02dZ",
                myGNSS.getYear(),    // Year
                myGNSS.getMonth(),   // Month
                myGNSS.getDay(),     // Day
                myGNSS.getHour(),    // Hour (assuming UTC+4)
                myGNSS.getMinute(),  // Minute
                myGNSS.getSecond()); // Second
    }

    GPS_DBG.println(dt);
    return String(dt);
}

double myGPS::getLatitude()
{
    return isValid ? myGNSS.getLatitude() / 10000000.0 : 0.0;
}

double myGPS::getLongitude()
{
    return isValid ? myGNSS.getLongitude() / 10000000.0 : 0.0;
}

double myGPS::getAltitude()
{
    return isValid ? myGNSS.getAltitude() / 1000.0 : 0.0;
}

double myGPS::getSpeedKM()
{
    if(isValid){

        float speed_KmHr = (myGNSS.getGroundSpeed() / 1000000.0)* 3600.0;
        return speed_KmHr; 
    }
    return 0.0;
}

uint32_t myGPS::getStatlites()
{
    return isValid ? myGNSS.getSIV() : 0;
}

uint8_t myGPS::getFix()
{
    return myGNSS.getFixType();
}
void myGPS::readablePrintEx()
{
    char spBuf[255];

    sprintf(spBuf, "[GPS] Fix: %d , Lat: %0.6f , Long: %0.6f , Alt: %0.6f M, Speed: %0.2f KM , SATs: %d  ",
            getFix(),
            (float)getLatitude(),
            (float)getLongitude(),
            (float)getAltitude(),
            (float)getSpeedKM(),
            getStatlites());

    GPS_DBG.println(spBuf);
}