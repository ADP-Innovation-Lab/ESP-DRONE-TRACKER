/*
  myGPS.h - Library for interfacing with GPS module.
            supports both ESP32 and AVR boards
  Created by [Mohamed Maher ]
  Dependency:
        http://librarymanager/All#SparkFun_u-blox_GNSS
*/

#ifndef UBXGPS_H
#define UBXGPS_H

#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS

// GPS UART PORT CONFIG -
#define GPSBaud 38400  // 38400 IF UBLOX M9-MODULE OR 9600 FOR OTHERS
#define gpsTX   4        // 4        // 17
#define gpsRX   15       // 15       // 16
#define gpsPORT 1      // 1        // 2
#define GPS_DBG Serial // serial 0 for logging

class myGPS
{
public:
    void setup();
    void loop();
    void readablePrintEx();
    String getDateTimeStr();
    double getLatitude();
    double getLongitude();
    double getAltitude();
    double getSpeedKM();
    uint32_t getStatlites();
    uint8_t getFix();

private:
};

#endif