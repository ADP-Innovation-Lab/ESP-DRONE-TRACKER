#include <Arduino.h>
#include "myLTE.h"
#include <ArduinoJson.h>
#include "myGPS.h"
#include "myIMU.h"
myGPS witGPS;

//------------- varaibles and defs
uint8_t rtAttemp = 5;

DynamicJsonDocument doc(MQTT_BUFFER_ZISE);

unsigned long lastPublishTime = 0;
const unsigned long publishInterval = 10000; // Publish payload every 5 seconds
//-------------- functions
String create_jsonPayload();

void setup()
{
  Serial.begin(115200);
  lte_setup();
  witGPS.setup();
  imu_setup();

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

  imu_loop();
  imu_print();

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
  imu["roll"]  = imu_getRoll(); 
  imu["pitch"] = imu_getPitch();
  imu["yaw"]   = imu_getYaw();
  doc["lastFlightStart"] = "";
  doc["lastFlightStop"] = "";
  doc["speed"] = witGPS.getSpeed();
  doc["lteSignal"] = lte_getSignalQuality();

  // Serialize JSON to string
  String payload;
  serializeJson(doc, payload);
  DBG("[APP] Payload Length in Bytes: ", strlen(payload.c_str()));

  // Debugging: Print the payload to Serial Monitor
  // SerialMon.println("Generated JSON Payload:");
  // SerialMon.println(payload);

  return payload;
}