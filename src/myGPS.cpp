/*
  my_gps.cpp - Library for interfacing with GPS module.
            supports both ESP32 and AVR boards
  Created by [Mohamed Maher ]
  Dependency:
        https://github.com/mikalhart/TinyGPSPlus
*/
#include "myGPS.h"

#ifdef __AVR__
#include <SoftwareSerial.h>
SoftwareSerial gpsSerial(gpsRX, gpsTX);
#else
#include <HardwareSerial.h>
HardwareSerial gpsSerial(gpsPORT);
#endif

void myGPS::setup()
{
#ifdef __AVR__
    gpsSerial.begin(GPSBaud);
#else
    gpsSerial.begin(GPSBaud, SERIAL_8N1, gpsRX, gpsTX);
#endif
    GPS_DBG.println(F("[GPS] Serial port init success."));
    // Add your setup code here# Tatweer broker
}

void myGPS::loop()
{
    smartDelay(1000); // Delay for 5 seconds
    if (millis() > 5000 && _gps.charsProcessed() < 10)
        GPS_DBG.println(F("[GPS] No GPS data received: check wiring"));
}

String myGPS::getPayload()
{
    // Construct the payload string
    String payload = String(_gps.date.month()) + "/" + // Date
                     String(_gps.date.day()) + "/" +
                     String(_gps.date.year()) + "," +
                     String(_gps.time.hour() + 4) + ":" + // Time UTC+4
                     String(_gps.time.minute()) + ":" +
                     String(_gps.time.second()) + "," +
                     String(_gps.location.lat(), 6) + "," +    // Latitude
                     String(_gps.location.lng(), 6) + "," +    // Longitude
                     String(_gps.altitude.meters(), 2) + "," + // Altitude in Meters
                     String(_gps.speed.kmph(), 2) + "," +      // Speed
                     String(_gps.satellites.value()) + "," +   //
                     String(_gps.hdop.hdop(), 1) + "," +
                     String(_gps.location.age()) + "," +
                     String(_gps.course.deg(), 2) + "," +
                     String(_gps.charsProcessed()) + "," +
                     String(_gps.sentencesWithFix()) + "," +
                     String(_gps.failedChecksum());

    return payload;
}

String myGPS::getDateTimeStr()
{
    // expected : "2024-05-04T14:43:22Z"
    char dt[25]; 
    // Format the date and time components with leading zeros
    if( _gps.date.year() < 2024){
          sprintf(dt, "%04d-%02d-%02dT%02d:%02d:%02dZ",
            2024, // Year
            01, // Month
            01, // Day
            _gps.time.hour() + 4, // Hour (assuming UTC+4)
            _gps.time.minute(), // Minute
            _gps.time.second()); // Second  
    }
    else{
        sprintf(dt, "%04d-%02d-%02dT%02d:%02d:%02dZ",
            _gps.date.year(), // Year

            _gps.date.month(), // Month
            _gps.date.day(), // Day
            _gps.time.hour() + 4, // Hour (assuming UTC+4)
            _gps.time.minute(), // Minute
            _gps.time.second()); // Second
    }
    

    //GPS_DBG.println(dt); 
    return String(dt); 
}
double myGPS::getLatitude()
{
    return _gps.location.lat();
}

double myGPS::getLongitude()
{
    return _gps.location.lng();
}

double myGPS::getAltitude()
{
    return _gps.altitude.meters();
}

double myGPS::getSpeed()
{
    return _gps.speed.kmph();
}

uint32_t myGPS::getStatlites()
{
    return _gps.satellites.value();
}

//---------- private methods

void myGPS::printStr(const char *str, int len)
{
    int slen = strlen(str);
    for (int i = 0; i < len; ++i)
        GPS_DBG.print(i < slen ? str[i] : ' ');
    smartDelay(0);
}

void myGPS::printFloat(float val, bool valid, int len, int prec)
{
    if (!valid)
    {
        while (len-- > 1)
            GPS_DBG.print('*');
        GPS_DBG.print(' ');
    }
    else
    {
        GPS_DBG.print(val, prec);
        int vi = abs((int)val);
        int flen = prec + (val < 0.0 ? 2 : 1); // . and -
        flen += vi >= 1000 ? 4 : vi >= 100 ? 3
                             : vi >= 10    ? 2
                                           : 1;
        for (int i = flen; i < len; ++i)
            GPS_DBG.print(' ');
    }
    smartDelay(0);
}

void myGPS::printInt(unsigned long val, bool valid, int len)
{
    char sz[32] = "*****************";
    if (valid)
        sprintf(sz, "%ld", val);
    sz[len] = 0;
    for (int i = strlen(sz); i < len; ++i)
        sz[i] = ' ';
    if (len > 0)
        sz[len - 1] = ' ';
    GPS_DBG.print(sz);
    smartDelay(0);
}

void myGPS::printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
    if (!d.isValid())
    {
        GPS_DBG.print(F("********** "));
    }
    else
    {
        char sz[32];
        sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
        GPS_DBG.print(sz);
    }

    if (!t.isValid())
    {
        GPS_DBG.print(F("******** "));
    }
    else
    {
        char sz[32];
        sprintf(sz, "%02d:%02d:%02d ", t.hour() + 4, t.minute(), t.second()); // UTC+4
        GPS_DBG.print(sz);
    }

    printInt(d.age(), d.isValid(), 5);
    smartDelay(0);
}

void myGPS::smartDelay(unsigned long ms)
{
    unsigned long start = millis();
    do
    {
        while (gpsSerial.available())
            _gps.encode(gpsSerial.read());
    } while (millis() - start < ms);
}

void myGPS::readablePrint()
{
    static const double AD_LAT = 51.508131, AD_LON = -0.128002;

    printInt(_gps.satellites.value(), _gps.satellites.isValid(), 5);
    printFloat(_gps.hdop.hdop(), _gps.hdop.isValid(), 6, 1);
    printFloat(_gps.location.lat(), _gps.location.isValid(), 11, 6);
    printFloat(_gps.location.lng(), _gps.location.isValid(), 12, 6);
    printInt(_gps.location.age(), _gps.location.isValid(), 5);
    printDateTime(_gps.date, _gps.time);
    printFloat(_gps.altitude.meters(), _gps.altitude.isValid(), 7, 2);
    printFloat(_gps.course.deg(), _gps.course.isValid(), 7, 2);
    printFloat(_gps.speed.kmph(), _gps.speed.isValid(), 6, 2);
    printStr(_gps.course.isValid() ? TinyGPSPlus::cardinal(_gps.course.deg()) : "*** ", 6);

    unsigned long distanceKmToLondon =
        (unsigned long)TinyGPSPlus::distanceBetween(
            _gps.location.lat(),
            _gps.location.lng(),
            AD_LAT,
            AD_LON) /
        1000;
    printInt(distanceKmToLondon, _gps.location.isValid(), 9);

    double courseToLondon =
        TinyGPSPlus::courseTo(
            _gps.location.lat(),
            _gps.location.lng(),
            AD_LAT,
            AD_LON);

    printFloat(courseToLondon, _gps.location.isValid(), 7, 2);

    const char *cardinalToLondon = TinyGPSPlus::cardinal(courseToLondon);

    printStr(_gps.location.isValid() ? cardinalToLondon : "*** ", 6);

    printInt(_gps.charsProcessed(), true, 6);
    printInt(_gps.sentencesWithFix(), true, 10);
    printInt(_gps.failedChecksum(), true, 9);
    GPS_DBG.println();
}

void myGPS::readablePrintEx()
{
    char spBuf[255];
    sprintf(spBuf, "[GPS] HDOP: %0.1f , Lat: %0.6f , Long: %0.6f , Alt: %0.6f M, Speed: %0.2f KM , SATs: %d  ",
            (_gps.hdop.isValid()) ? (float)_gps.hdop.hdop() : 0.0,
            (_gps.location.isValid()) ? (float)_gps.location.lat() : 0.00,
            (_gps.location.isValid()) ? (float)_gps.location.lng() : 0.00,
            (_gps.altitude.isValid()) ? _gps.altitude.meters() : 0.00,
            (_gps.speed.isValid()) ? (float)_gps.speed.kmph() : 0.00,
            (_gps.satellites.isValid()) ? _gps.satellites.value() : 0);
    GPS_DBG.println(spBuf);
}