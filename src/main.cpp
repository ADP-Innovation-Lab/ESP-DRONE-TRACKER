#include <Arduino.h>
#include "myLTE.h"
#include <ArduinoJson.h>
#include "myGPS.h"
#include "sensors.h"
#include <WiFi.h>
#include "esp_task_wdt.h"
#include "device_info.h"
#include "state.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

myGPS ubxM6;

//------------ freeRTOS defs
TaskHandle_t AppHandlde = NULL;
TaskHandle_t sensorsHandle = NULL;
TaskHandle_t interruptHandle = NULL;

SemaphoreHandle_t ints_xSemaphore = NULL;

//------------- varaibles and defs
uint8_t rtAttemp = 5;
DynamicJsonDocument doc(MQTT_BUFFER_ZISE);
unsigned long lastPublishTime = 0;

const unsigned long publishInterval = 5000; // Publish payload every 5 seconds

#define LED_STATUS 2
#define LTE_PWRKEY 12
#define LTE_STATUS 39

//-------------- functions
void state_sensors_read(SensorData *sensor_data);
String create_jsonPayload();
void appTask(void *pvParameters);
void sensorsTask(void *pvParameters);
//------------- State Variables
SensorData sensor_data;
StateData state_data;
char state = 'S'; // Initial state is stopped

char *dataEvent = "DataUpdate";
char *droneState = "Stopped";

void setup()
{

  Serial.begin(115200);
  pinMode(LTE_PWRKEY, OUTPUT);
  pinMode(LED_STATUS, OUTPUT);
  pinMode(LTE_STATUS, INPUT);

  //-------------- Config GPIO interrupts
  pinMode(USB_STATUS_PIN, INPUT);
  pinMode(CHARGER_STATUS_PIN, INPUT);
  // pinMode(BOX_OPEN_PIN, INPUT);

  digitalWrite(LED_STATUS, LOW);

  // FOR TESTING - DIABLE WIFI
  WiFi.mode(WIFI_OFF);
  WiFi.disconnect(true);

  // Print device information
  Serial.printf("Device Information:\n");
  Serial.printf("-------------------\n");
  Serial.printf("MCU: %s\n", MCU);
  Serial.printf("GSM: %s\n", GSM);
  Serial.printf("GPS: %s\n", GPS);
  Serial.printf("IMU: %s\n", IMU);
  Serial.printf("Pressure Sensor: %s\n", PRESSURE_SENSOR);
  Serial.printf("PCB Version: %s\n", PCB_VERSION);
  Serial.printf("Firmware Version: %s\n", FIRMWARE_VERSION);
  Serial.printf("-------------------\n");

  delay(2000);
  // power on bg96
  // if it is already on
  if (digitalRead(LTE_STATUS) == HIGH)
  {
    Serial.println("[LTE] Modem is off, Powering on ...");

    digitalWrite(LTE_PWRKEY, HIGH);
    delay(800);
    digitalWrite(LTE_PWRKEY, LOW);
    delay(5000);
  }
  else
  {
    Serial.println("[LTE] Modem is already on!");
  }

  // create app task
  xTaskCreatePinnedToCore(
      appTask,     // Task function
      "appTask",   // Task name
      15000,       // Stack size (bytes)
      NULL,        // Task parameters
      1,           // Priority (1 is highest priority)
      &AppHandlde, // Task handle
      0            // Core to run the task on (0 or 1)
  );

  // create app task
  xTaskCreatePinnedToCore(
      sensorsTask,    // Task function
      "sensorsTask",  // Task name
      15000,          // Stack size (bytes)
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

  lte_led_init();

  if (!lte_setup())
  {
    DBG("[APP] LTE modem init failed ... restarting ");
    delay(2000);
    esp_restart();
  }

  lte_getSignalQuality();
  lte_led_update();
  ubxM6.setup();

  while (!lte_connect() && rtAttemp > 0)
  {
    rtAttemp--;
    delay(2000);

    DBG("LTE reconnect attemp no : ", rtAttemp);
  }

  if (lte_mqttSetup())
    digitalWrite(LED_STATUS, HIGH);

  while (1)
  {
    ubxM6.loop();
    ubxM6.readablePrintEx();

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
    lte_led_update();

    vTaskDelay(500 / portTICK_PERIOD_MS);
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
  battery_setup();

  while (1)
  {
    imu_loop();
    //  Periodic stuff
    if (millis() - preriodiMills >= 3000)
    {
      preriodiMills = millis();
      battery_read();
      lte_getSignalQuality();
      imu_print();
      bmp_print();

      // Check low battery event:
      if (battery_getStorage() < 20)
        dataEvent = "LowBattery";
      else
        dataEvent = "DataUpdate";

      // Check for anti-tempering

      // Get drone State
      state_init(&state_data,
                 ubxM6.getLatitude(),
                 ubxM6.getLongitude(),
                 bmp_getRelativeAltitude());
      delay(1000);
      state_sensors_read(&sensor_data);

      state_update(&state_data, &sensor_data, &state);
      if (state == 'F')
      {
        droneState={0}; 
        droneState = "Flying";
        Serial.println("[State] Drone is flying\n");
      }
      else
      {
        droneState={0}; 
        droneState = "Stopped";
        Serial.println("[State] Drone is Stopped\n");
       
      }
    }

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

// Mock function to read sensor data
void state_sensors_read(SensorData *sensor_data)
{
  // Populate with actual sensor reading code
  sensor_data->accel_x = imu_getAccelX();
  sensor_data->accel_y = imu_getAccelY();
  sensor_data->accel_z = imu_getAccelZ();
  sensor_data->altitude = bmp_getRelativeAltitude();
  sensor_data->speed = ubxM6.getSpeed();
  sensor_data->latitude = ubxM6.getLatitude();
  sensor_data->longitude = ubxM6.getLongitude();
}
/**
 * @brief Create a jsonPayload object
 *
 * @return String
 */
String create_jsonPayload()
{
  doc["id"] = "DT101";                       // Use device IMEI as ID
  doc["timestamp"] = ubxM6.getDateTimeStr(); // modem.getGSMDateTime(DATE_FULL); // Get modem time for timestamp
  doc["latitude"] = ubxM6.getLatitude();
  doc["longitude"] = ubxM6.getLongitude();

  JsonObject deviceData = doc["deviceData"].to<JsonObject>();
  deviceData["status"] = droneState;
  deviceData["event"] = dataEvent;

  JsonObject battery = deviceData["battery"].to<JsonObject>();
  battery["voltage"] = battery_getVoltage();
  battery["percentage"] = battery_getStorage();

  JsonObject location = deviceData["location"].to<JsonObject>();
  location["latitude"] = ubxM6.getLatitude();
  location["longitude"] = ubxM6.getLongitude();
  location["altitude"] = bmp_getRelativeAltitude(); // witGPS.getAltitude();

  JsonObject imu = deviceData["imu"].to<JsonObject>();
  imu["roll"] = imu_getRoll();
  imu["pitch"] = imu_getPitch();
  imu["yaw"] = imu_getYaw();

  deviceData["lastFlightStart"] = ubxM6.getDateTimeStr();
  ;
  deviceData["lastFlightStop"] = ubxM6.getDateTimeStr();
  ;
  deviceData["speed"] = ubxM6.getSpeed();
  deviceData["lteSignal"] = lte_getSignalQuality();
  deviceData["sats"] = ubxM6.getStatlites();

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
