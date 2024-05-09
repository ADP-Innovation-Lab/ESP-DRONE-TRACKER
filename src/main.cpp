#include <Arduino.h>
#include "myLTE.h"
#include <ArduinoJson.h>
#include "ubxGPS.h"
#include "sensors.h"
#include <WiFi.h>
myGPS ubxM9;

//------------ freeRTOS defs
TaskHandle_t AppHandlde = NULL;
TaskHandle_t sensorsHandle = NULL;

//------------- varaibles and defs
uint8_t rtAttemp = 5;
DynamicJsonDocument doc(MQTT_BUFFER_ZISE);
unsigned long lastPublishTime = 0;
const unsigned long publishInterval = 2000; // Publish payload every 5 seconds

#define LED_STATUS 2
#define LTE_PWRKEY 18

//-------------- functions
String create_jsonPayload();
void appTask(void *pvParameters);
void sensorsTask(void *pvParameters);

void setup()
{

  Serial.begin(115200);
  pinMode(LTE_PWRKEY, OUTPUT);
  pinMode(LED_STATUS, OUTPUT);
  digitalWrite(LED_STATUS, LOW);

  // FOR TESTING - DIABLE WIFI
  WiFi.mode(WIFI_OFF);
  WiFi.disconnect(true);

  // power on bg96
  delay(2000);
  digitalWrite(LTE_PWRKEY, HIGH);
  delay(800); // Core to run the task on (0 or 1)
  digitalWrite(LTE_PWRKEY, LOW);
  delay(5000);

  // create app task
  xTaskCreatePinnedToCore(
      appTask,     // Task function
      "appTask",   // Task name
      10000,       // Stack size (bytes)
      NULL,        // Task parameters
      1,           // Priority (1 is highest priority)
      &AppHandlde, // Task handle
      0            // Core to run the task on (0 or 1)
  );

  // create app task
  xTaskCreatePinnedToCore(
      sensorsTask,    // Task function
      "sensorsTask",  // Task name
      10000,          // Stack size (bytes)
      NULL,           // Task parameters
      1,              // Priority (1 is highest priority)
      &sensorsHandle, // Task handle
      1               // Core to run the task on (0 or 1)
  );
}

void loop()
{
}

/**
 * Main App thread , includes LTE, Mqtt and GPS
 *
 * @param pvParameters
 */

void appTask(void *pvParameters)
{
  DBG("[APP] App thead init ...");
  if (!lte_setup())
  {
    DBG("[APP] LTE modem init failed ... restarting ");
    delay(2000);
    // should restart esp here !
  }
  ubxM9.setup();

  while (!lte_connect() && rtAttemp > 0)
  {
    rtAttemp--;
    delay(5000);
    DBG("LTE reconnect attemp no : %d", rtAttemp);
  }

  if (lte_mqttSetup())
    digitalWrite(LED_STATUS, HIGH);

  while (1)
  {
    ubxM9.loop();
    ubxM9.readablePrintEx();

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
}
/**
 * @brief Sensors fusion thread, includes IMU, BMP, Anti-tampring and Battery
 *
 * @param pvParameters
 */
void sensorsTask(void *pvParameters)
{

  unsigned long preriodiMills = 0;
  DBG("[APP] Sensors thead init ...");
  imu_setup();
  bmp_setup();

  while (1)
  {
    imu_loop();
    bmp_loop();

    // Periodic stuff
    if (millis() - preriodiMills >= 2000)
    {
      preriodiMills = millis();
      imu_print();
      bmp_print();
    }

    delay(50);
  }
}

/**
 * @brief Create a jsonPayload object
 *
 * @return String
 */
String create_jsonPayload()
{
  doc["id"] = "DT101";                       // Use device IMEI as ID
  doc["timestamp"] = ubxM9.getDateTimeStr(); // modem.getGSMDateTime(DATE_FULL); // Get modem time for timestamp
  doc["latitude"] = ubxM9.getLatitude();
  doc["longitude"] = ubxM9.getLongitude();

  JsonObject deviceData = doc["deviceData"].to<JsonObject>();
  deviceData["status"] = "Flying";
  deviceData["event"] = "DataUpdate";

  JsonObject battery = deviceData["battery"].to<JsonObject>();
  battery["voltage"] = 3.7;
  battery["percentage"] = 80;

  JsonObject location = deviceData["location"].to<JsonObject>();
  location["latitude"] = ubxM9.getLatitude();
  location["longitude"] = ubxM9.getLongitude();
  location["altitude"] = bmp_act_altitude(); // witGPS.getAltitude();

  JsonObject imu = deviceData["imu"].to<JsonObject>();
  imu["roll"] = imu_getRoll();
  imu["pitch"] = imu_getPitch();
  imu["yaw"] = imu_getYaw();

  deviceData["lastFlightStart"] = ubxM9.getDateTimeStr();
  ;
  deviceData["lastFlightStop"] = ubxM9.getDateTimeStr();
  ;
  deviceData["speed"] = ubxM9.getSpeedKM();
  deviceData["lteSignal"] = lte_getSignalQuality();
  deviceData["sats"] = ubxM9.getStatlites();

  /*
  doc["lastFlightStart"] = "";
  doc["lastFlightStop"] = "";
  doc["speed"] = witGPS.getSpeed();
  doc["lteSignal"] = lte_getSignalQuality();
  doc["sats"] = witGPS.getStatlites();
  */
  // Serialize JSON to string
  String payload;
  serializeJson(doc, payload);
  DBG("[APP] Payload Length in Bytes: ", strlen(payload.c_str()));

  // Debugging: Print the payload to Serial Monitor
  // SerialMon.println("Generated JSON Payload:");
  // SerialMon.println(payload);

  return payload;
}