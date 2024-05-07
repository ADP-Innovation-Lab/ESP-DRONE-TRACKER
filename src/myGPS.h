/*
  myGPS.h - Library for interfacing with GPS module. 
            supports both ESP32 and AVR boards 
  Created by [Mohamed Maher ]
  Dependency:
        https://github.com/mikalhart/TinyGPSPlus
*/


#ifndef MYGPS_H
#define MYGPS_H


#include <Arduino.h>
#include <TinyGPS++.h>


// GPS UART PORT CONFIG - 
#define GPSBaud 9600     // 34800 IF UBLOX M9-MODULE OR 9600 FOR OTHERS 
#define gpsTX   4        //17
#define gpsRX   15       //16
#define gpsPORT 1       //1
#define GPS_DBG     Serial   // serial 0 for logging 

class myGPS {
public:
    void setup();
    void loop();
    String getPayload();
    void readablePrint(); 
    void readablePrintEx(); 
    String getDateTimeStr();
    double getLatitude(); 
    double getLongitude();
    double getAltitude(); 
    double getSpeed();
    uint32_t getStatlites(); 

private:
    TinyGPSPlus _gps;
    String payload;
    

    void printStr(const char *str, int len);
    void printFloat(float val, bool valid, int len, int prec);
    void printInt(unsigned long val, bool valid, int len);
    void printDateTime(TinyGPSDate &d, TinyGPSTime &t);
    void smartDelay(unsigned long ms);
};

#endif