#include <Arduino.h>
#include "myLTE.h"
#include <ArduinoJson.h>
#include "myGPS.h"

myGPS witGPS;

//------------- varaibles and defs
uint8_t rtAttemp = 5;

DynamicJsonDocument doc(512);

unsigned long lastPublishTime = 0;
const unsigned long publishInterval = 10000; // Publish payload every 5 seconds
//-------------- functions
String create_jsonPayload();

void setup()
{
  Serial.begin(115200);
  lte_setup();
  witGPS.setup();

  while (!lte_connect() && rtAttemp > 0)
  {
    rtAttemp--;
    delay(5000);
    DBG("LTE reconnect attemp no : %d", rtAttemp);
  }

  lte_mqttSetup();
}

void loop()
{

  witGPS.loop();
  witGPS.readablePrintEx();
  // Periodic payload publish
  unsigned long currentMillis = millis();
  if (currentMillis - lastPublishTime >= publishInterval)
  {
    lastPublishTime = currentMillis;
    if (lte_isMqttConnected())
    {
      lte_mqttPublish(create_jsonPayload());
      DBG("[APP] MQTT Payload published ");
    }
    else
    {
      lte_mqttReconnect();
      DBG("[APP] MQTT Payload NOT published ");
    }
  }
  lte_mqttLoop();
  delay(500);
}

String create_jsonPayload()
{

  doc["id"] = "DT101";                     // Use device IMEI as ID
  doc["timestamp"]  = witGPS.getDateTimeStr(); // modem.getGSMDateTime(DATE_FULL); // Get modem time for timestamp
  doc["latitude"]   = witGPS.getLatitude();
  doc["longitude"]  = witGPS.getLongitude();

  JsonObject deviceData = doc["deviceData"].to<JsonObject>();
  deviceData["status"] = "Flying";
  deviceData["event"]  = "DataUpdate";

  JsonObject battery = deviceData["battery"].to<JsonObject>();
  battery["voltage"] = 3.7;
  battery["percentage"] = 80;

  JsonObject location = deviceData["location"].to<JsonObject>();
  location["latitude"]  = witGPS.getLatitude();
  location["longitude"] = witGPS.getLongitude();
  location["altitude"]  = witGPS.getAltitude();

  JsonObject imu = deviceData["imu"].to<JsonObject>();
  JsonObject acceleration = imu["acceleration"].to<JsonObject>();
  acceleration["x"] = 0.1;
  acceleration["y"] = -0.2;
  acceleration["z"] = 9.8;

  JsonObject gyroscope = imu["gyroscope"].to<JsonObject>();
  gyroscope["x"] = 10.5;
  gyroscope["y"] = -5.2;
  gyroscope["z"] = 3.0;

  JsonObject magnetometer = imu["magnetometer"].to<JsonObject>();
  magnetometer["x"] = -30.2;
  magnetometer["y"] = 20.1;
  magnetometer["z"] = -15.8;

  doc["lastFlightStart"] = "";
  doc["lastFlightStop"] = "";
  doc["speed"] = witGPS.getSpeed();
  doc["lteSignal"] = 21;

  // Serialize JSON to string
  String payload;
  serializeJson(doc, payload);
  DBG("[APP] Payload Length in Bytes: ", strlen(payload.c_str()));

  // Debugging: Print the payload to Serial Monitor
  // SerialMon.println("Generated JSON Payload:");
  // SerialMon.println(payload);

  return payload;
}