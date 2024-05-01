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
#define MQTT_BUFFER_ZISE 512

//------------------- Global Variables
String deviceImei;
uint32_t lastReconnectAttempt = 0;

void lte_setup()
{

#ifdef TINY_GSM_AUTOBAUAD_ENABLE
    SerialAT.begin(MODEM_RX, MODEM_TX);
    SerialMon.println("Wait...");
    delay(5000); // wait for modem to power up
    // Set GSM module baud rate
    TinyGsmAutoBaud(SerialAT, GSM_AUTOBAUD_MIN, GSM_AUTOBAUD_MAX);
#elif
    SerialAT.begin(GSM_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX)
#endif
    delay(6000);
    SerialMon.println("Initializing modem...");

    modem.restart();
    // modem.init();
    DBG("======= Modem Info =========");
    String modemInfo = modem.getModemInfo();

    DBG("[+] Modem:", modemInfo);
    SerialMon.println(modemInfo);

    String ccid = modem.getSimCCID();
    DBG("[+] CCID:", ccid);

    deviceImei = modem.getIMEI();
    DBG("[+] IMEI:", deviceImei);

    String imsi = modem.getIMSI();
    DBG("[+] IMSI:", imsi);

    // String cop = modem.getOperator();
    // DBG("[+] Operator:", cop);

    IPAddress local = modem.localIP();
    DBG("[+] Local IP:", local);

    int csq = modem.getSignalQuality();
    DBG("[+] Signal quality:", csq);
    DBG("===========================");
}

boolean lte_connect()
{
    SerialMon.print("[LTE] Waiting for network...");
    if (!modem.waitForNetwork())
    {
        SerialMon.println(" fail");
        delay(10000);
        // return false;
    }
    SerialMon.println(" success");

    if (modem.isNetworkConnected())
    {
        SerialMon.println("[LTE] Network connected");
    }
    else
    {
        SerialMon.println("[LTE] Network Not COnnected");
        return false;
    }

    // GPRS connection parameters are usually set after network registration
    SerialMon.print(F("[LTE] Connecting to "));
    SerialMon.print(apn);
    if (!modem.gprsConnect(apn, gprsUser, gprsPass))
    {
        SerialMon.println(" fail");
        delay(10000);
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
    modem.sendAT(GF("+QCSQ"));
    if (modem.waitResponse(GF("+QCSQ:")) != 1)
    {
        return 99;
    }
    // int8_t res = thisModem().streamGetIntBefore(',');
    // thisModem().waitResponse();
    // need to make custom imp current one not working
    return 21;
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

    // Connect to MQTT Broker
    boolean status = mqtt.connect(clientId.c_str());
    mqtt.setBufferSize(MQTT_BUFFER_ZISE);
    return status;
}

void lte_mqttReconnect()
{
    if (!mqtt.connected())
    {
        SerialMon.println("=== MQTT NOT CONNECTED ===");
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
}

boolean lte_isMqttConnected()
{
    return mqtt.connected();
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

String getUniqueClientId()
{
    String imei = deviceImei;
    uint32_t currentMillis = millis();
    return imei + String(currentMillis);
}
