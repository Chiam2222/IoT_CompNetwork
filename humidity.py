import os
import time
import json
import adafruit_dht
import board
import paho.mqtt.client as mqtt

# Define callback functions for MQTT events
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    if rc == 0:
        print("Connection successful")
    else:
        print("Connection failed")

    # Subscribe to a topic when connected
    client.subscribe("topic/to/subscribe")

def on_message(client, userdata, msg):
    print(msg.topic + " " + str(msg.payload))

# Define MQTT broker address, port, username, and password
broker_address = "homeassistant"
port = 1883
username = "koi"
password = "12345678Uni"
publish_topic = "sarahpi/sensor"  # Replace with your desired topic

# Create a MQTT client instance
client = mqtt.Client()

# Assign callbacks for MQTT events
client.on_connect = on_connect
client.on_message = on_message

# Set username and password
client.username_pw_set(username, password)

# Connect to MQTT broker
client.connect(broker_address, port, 60)

# Start the MQTT loop to handle communication
client.loop_start()

# Initialize DHT22 sensor
dht_device = adafruit_dht.DHT22(board.D4)

# File path for storing sensor data
file_path = '/home/pi/dht22/humidity.csv'
os.makedirs(os.path.dirname(file_path), exist_ok=True)

with open(file_path, 'a+') as f:
    f.seek(0)
    if os.stat(file_path).st_size == 0:
        f.write('Date,Time,Temperature C,Humidity\r\n')
    f.seek(0, os.SEEK_END)

    while True:
        try:
            temperature_c = dht_device.temperature
            humidity = dht_device.humidity
            print('Temp: {:.1f} C Humidity: {:.1f}%'.format(temperature_c, humidity))

            # Create JSON object
            sensor_data = {
                "temperature": temperature_c,
                "humidity": humidity
            }

            # Convert JSON object to string
            json_data = json.dumps(sensor_data)

            # Publish sensor data to MQTT topic
            client.publish(publish_topic, json_data)

            # Write sensor data to CSV file
            f.write('{0},{1},{2:0.1f}*C,{3:0.1f}%\r\n'.format(time.strftime('%m/%d/%y'), time.strftime('%H:%M'), temperature_c, humidity))
            f.flush()
        except RuntimeError as err:
            print(err.args[0])

        time.sleep(2)

