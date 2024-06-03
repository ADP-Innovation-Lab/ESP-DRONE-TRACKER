#ifndef MYLTE_H
#define MYLTE_H

#include <Arduino.h>
#include <FastLED.h>


// config  your modem:
#define TINY_GSM_MODEM_BG96
#define SerialMon Serial
#define TINY_GSM_DEBUG SerialMon
#define SerialAT Serial2 // Modem Serial

// Select serial2
#define MODEM_TX 17
#define MODEM_RX 16

//#define TINY_GSM_AUTOBAUAD_ENABLE
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Select Modem Buadrate settings

#ifdef TINY_GSM_AUTOBAUAD_ENABLE
#define GSM_AUTOBAUD_MIN 9600
#define GSM_AUTOBAUD_MAX 115200
#else
#define GSM_BAUD 115200
#endif

#define MQTT_BUFFER_ZISE 1024

//LTE LED Variables 
#define LTE_LED_PIN 25  
#define NUM_LEDS 1 
#define LED_INTENSITY 32 


//------------- GSM functions 
boolean lte_setup();
boolean lte_connect();
void lte_restart(); 
String lte_getGSMDateTime();
int8_t lte_getSignalQuality(); 
void lte_turnOn(); 
void lte_turnOff(); 
boolean lte_reconnect(uint8_t atemps); 

//------------ Mqtt Functions
boolean lte_mqttSetup(); 
void lte_mqttReconnect(); 
boolean lte_isMqttConnected(); 
void lte_mqttLoop(); 
boolean lte_mqttPublish( String payload);
void lte_mqttCallback(char *topic, byte *payload, unsigned int len);


//-------------- LTE LED functions 
void lte_led_init(); 
void lte_led_update(); 

//----------- Helper functions
String getUniqueClientId();



#endif
