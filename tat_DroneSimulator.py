import paho.mqtt.client as mqtt
import json
import time

# Test broker 
#broker_address = "broker.hivemq.com"
#port = 1883

# Tatweer broker 
broker_address = "tatweer.fortiddns.com"
port = 48112

topic = "dts/drones/DT102/data"

initial_latitude  = 24.363535
initial_longitude = 54.353535
battery_capacity = 100
last_battery_update = time.time()

def publish_message(client, latitude, longitude, battery_percentage):
    event = "DataUpdate" if battery_percentage >= 20 else "LowBattery"
    msg = {
        "id": "DT102",
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "latitude": latitude,
        "longitude": longitude,
        "deviceData": {
            "status": "Flying",
            "event": event,
            "battery": {
                "voltage": 3.7,
                "percentage": battery_percentage
            },
            "location": {
                "latitude": latitude,
                "longitude": longitude,
                "altitude": 25.0
            },
            "imu": {
                "acceleration": {
                    "x": 0.1,
                    "y": -0.2,
                    "z": 9.8
                },
                "gyroscope": {
                    "x": 10.5,
                    "y": -5.2,
                    "z": 3.0
                },
                "magnetometer": {
                    "x": -30.2,
                    "y": 20.1,
                    "z": -15.8
                }
            },
            "lastFlightStart": "",
            "lastFlightStop": "",
            "speed": 10.5,
            "lteSignal": 22
        }
    }
    message = json.dumps(msg)
    client.publish(topic, message)
    print("Published message:", message)

def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT broker with result code " + str(rc))
    client.subscribe(topic)

def on_message(client, userdata, msg):
    print("Received message: " + msg.payload.decode())

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
client.on_connect = on_connect
client.on_message = on_message

client.connect(broker_address, port, 60)
client.loop_start()  # Start the loop to process incoming and outgoing messages

while True:
    current_time = time.time()
    if current_time - last_battery_update >= 60:
        last_battery_update = current_time
        if battery_capacity >= 10:
            battery_capacity -= 10
        else:
            battery_capacity = 0
    battery_percentage = battery_capacity
    initial_latitude += 0.1
    publish_message(client, initial_latitude, initial_longitude, battery_percentage)
    time.sleep(5)
