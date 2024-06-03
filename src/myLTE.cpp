#include "myLTE.h"

//------------------ config LTE variables
#define GSM_PIN ""
const char apn[] = "etisalat.ae";
const char gprsUser[] = "";
const char gprsPass[] = "";

TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);

//------------------ config mqtt
const char *broker = "tatweer.fortiddns.com";
const uint16_t port = 48112;

// const char *broker = "broker.hivemq.com";
// const uint16_t port = 1883;
String mqttDataTopic = "dts/drones/DT101/data";

//------------------- Global Variables
String deviceImei;
uint32_t lastReconnectAttempt = 0;
CRGB leds[NUM_LEDS];

bool isRSSI_FOUND = false;
bool isLTE_FOUND = false;
bool isMqqt_FOUND = false;

//------------- GSM functions
boolean lte_setup()
{

#ifdef TINY_GSM_AUTOBAUAD_ENABLE
    SerialAT.begin(MODEM_RX, MODEM_TX);
    SerialMon.println("Wait...");
    delay(5000); // wait for modem to power up
    // Set GSM module baud rate
    TinyGsmAutoBaud(SerialAT, GSM_AUTOBAUD_MIN, GSM_AUTOBAUD_MAX);
#else
    SerialAT.begin(GSM_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
#endif
    delay(6000);
    SerialMon.println("Initializing modem...");

    // modem.restart();
    if (!modem.init())
    {
        return false;
    }
    DBG("======= Modem Info =========");
    String modemInfo = modem.getModemInfo();

    DBG("[LTE] Modem:", modemInfo);
    SerialMon.println(modemInfo);

    String ccid = modem.getSimCCID();
    DBG("[LTE] CCID:", ccid);

    deviceImei = modem.getIMEI();
    DBG("[LTE] IMEI:", deviceImei);

    String imsi = modem.getIMSI();
    DBG("[LTE] IMSI:", imsi);

    // String cop = modem.getOperator();
    // DBG("[LTE] Operator:", cop);

    IPAddress local = modem.localIP();
    DBG("[LTE] Local IP:", local);

    int csq = modem.getSignalQuality();
    DBG("[LTE] Signal quality:", csq);
    DBG("===========================");
    return true;
}

boolean lte_connect()
{
    SerialMon.print("[LTE] Waiting for network...");
    if (!modem.waitForNetwork())
    {
        SerialMon.println(" fail");
        isLTE_FOUND = false;
        delay(5000);
        // return false;
    }
    SerialMon.println(" success");

    if (modem.isNetworkConnected())
    {
        SerialMon.println("[LTE] Network connected");
        isLTE_FOUND = true;
    }
    else
    {
        SerialMon.println("[LTE] Network Not COnnected");
        isLTE_FOUND = false;
        return false;
    }

    // GPRS connection parameters are usually set after network registration
    SerialMon.print(F("[LTE] Connecting to "));
    SerialMon.print(apn);
    if (!modem.gprsConnect(apn, gprsUser, gprsPass))
    {
        SerialMon.println(" fail");
        delay(5000);
        // return;
    }
    SerialMon.println(" success");

    if (modem.isGprsConnected())
    {
        SerialMon.println("[LTE] GPRS connected");
    }
    else
    {
        SerialMon.println("[LTE] GPRS not connected");
        return false;
    }
    return true;
}

void lte_restart()
{
    DBG("[LTE] Restarting Modem ... ");
    modem.restart();
}

String lte_getGSMDateTime()
{
    return modem.getGSMDateTime(DATE_FULL);
}

int8_t lte_getSignalQuality()
{
    int csq = modem.getSignalQuality();
    // DBG("[LTE] Signal quality:", csq);
    if (csq == 99)
    { // Mostly ANT is not connected or faulty
        DBG("[LTE] Signal quality: Not Known,", csq);
        /**
         * Should set LTE signl to red
         * check gps resitrations if failed , reset the BG96
         */
        isRSSI_FOUND = false;
    }
    else if (csq >= 13 && csq <= 16)
    {
        DBG("[LTE] Signal quality: MIDDLE,", csq);
        // lteSignalStrengthStatus = MIDDLE;
        isRSSI_FOUND = true;
    }
    else if (csq < 13 || csq > 31)
    {
        DBG("[LTE] Signal quality: POOR ,", csq);
        /**
         * Should set LTE signl to red
         * check gps resitrations if failed , reset the BG96
         */
        isRSSI_FOUND = false;
    }
    else if (csq > 20 && csq <= 31)
    {

        DBG("[LTE] Signal quality: EXCELENT,", csq);
        isRSSI_FOUND = true;
    }
    else
    {
        DBG("[LTE] Signal quality: GOOD,", csq);
        isRSSI_FOUND = true;
    }

    return csq;
}

boolean lte_reconnect(uint8_t atemps)
{

    while (!lte_connect() && atemps > 0)
    {
        atemps--;
        delay(5000);
        DBG("[LTE] reconnect attemp no : %d", atemps);
    }
    if (atemps > 0)
        return true;

    return false;
}

//-----------------------  Mqtt Functions ---------------------

boolean lte_mqttSetup()
{
    DBG("[LTE] Setting up Mqtt ");
    mqtt.setServer(broker, port);
    mqtt.setCallback(lte_mqttCallback);
    SerialMon.print("Connecting to : ");
    SerialMon.println(broker);
    // generate new client ID each time reconnect !
    String clientId = getUniqueClientId();
    CRGB leds[NUM_LEDS];
    // Connect to MQTT Broker
    boolean status = mqtt.connect(clientId.c_str());
    mqtt.setBufferSize(MQTT_BUFFER_ZISE);
    return status;
}

void lte_mqttReconnect()
{
    if (!mqtt.connected())
    {
        isMqqt_FOUND = false;
        SerialMon.println("[LTE]=== MQTT NOT CONNECTED ===");
        // Reconnect every 10 seconds
        uint32_t t = millis();
        if (t - lastReconnectAttempt > 10000L)
        {
            lastReconnectAttempt = t;
            if (lte_mqttSetup())
            {
                lastReconnectAttempt = 0;
            }
        }
        // delay(100);
        // return;
    }
    isMqqt_FOUND = true;
}

boolean lte_isMqttConnected()
{
    if(mqtt.connected()){
        isMqqt_FOUND = true;
        return true; 
    }
    isMqqt_FOUND = false;
    return false;
}

void lte_mqttLoop()
{
    mqtt.loop();
}

boolean lte_mqttPublish(String payload)
{
    return mqtt.publish(mqttDataTopic.c_str(), payload.c_str());
}

void lte_mqttCallback(char *topic, byte *payload, unsigned int len)
{
    SerialMon.print("Message arrived [");
    SerialMon.print(topic);
    SerialMon.print("]: ");
    SerialMon.write(payload, len);
    SerialMon.println();
}

//-------------- LTE LED functions
void lte_led_init()
{
    FastLED.addLeds<WS2812B, LTE_LED_PIN, GRB>(leds, NUM_LEDS);
    // turn it off
    leds[0] = CRGB(0, 0, 0); // RGB values (0-255)
    FastLED.show();
}

void lte_led_update()
{
    // all good - blue color
    if (isRSSI_FOUND && isLTE_FOUND && isMqqt_FOUND)
    {
        leds[0] = CRGB(0, 0, LED_INTENSITY); 
        FastLED.show();
    }
    // no mqtt - green color
    else if (isRSSI_FOUND && isLTE_FOUND && !isMqqt_FOUND)
    {
        leds[0] = CRGB(0, LED_INTENSITY, 0); 
        FastLED.show();
    }
    // no mqtt and no lte-  red color
    else if (isRSSI_FOUND && !isLTE_FOUND && !isMqqt_FOUND)
    {
        leds[0] = CRGB(LED_INTENSITY, 0, 0); 
        FastLED.show();
    }else if (isRSSI_FOUND ){
        leds[0] = CRGB(LED_INTENSITY, 0, 0); 
        FastLED.show();
    }
    else{
        leds[0] = CRGB(LED_INTENSITY, LED_INTENSITY, LED_INTENSITY); 
        FastLED.show();
    }
}

//----------- Helper functions
String getUniqueClientId()
{
    String imei = deviceImei;
    uint32_t currentMillis = millis();
    return imei + String(currentMillis);
}
