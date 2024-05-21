import RPi.GPIO as GPIO
import time
import threading
import busio
import digitalio
import board    
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import json
import paho.mqtt.client as mqtt

# Define callback functions for MQTT events
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    if rc == 0:
        print("Connection successful")
    else:
        print("Connection failed")

    # Subscribe to relevant topics when connected
    client.subscribe("xunyinpi/pump_control")

def on_message(client, userdata, msg):
    print(msg.topic + " " + str(msg.payload))
    if msg.topic == "xunyinpi/pump_control":
        if msg.payload.decode() == "ON":
            manual_control(True)
        elif msg.payload.decode() == "OFF":
            manual_control_off()

# MQTT broker details
broker_address = 'homeassistant'
port = 1883
username = 'koi'
password = '12345678Uni'
publish_topic = 'xunyinpi/sensor'  # Replace with desired topic

# Create MQTT client instance
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

# GPIO setup for the pump
SOIL_MOISTURE_SENSOR_PIN = 21
RELAY_CONTROL_PIN = 15
GPIO.setmode(GPIO.BCM)
GPIO.setup(SOIL_MOISTURE_SENSOR_PIN, GPIO.IN)
GPIO.setup(RELAY_CONTROL_PIN, GPIO.OUT)

# SPI setup for the soil moisture sensor
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
cs = digitalio.DigitalInOut(board.CE0)
mcp = MCP.MCP3008(spi, cs)
chan = AnalogIn(mcp, MCP.P0)    # dry soil adc value ~64742

threshold_value = 40
pump_on_flag = False
manual_pump_on = False

def soil_moisture_level_low():  # soil dry, value high; soil wet, value low
    moisture = round(100 - ((chan.value / 70000) * 100), 2)
    return moisture < threshold_value

def manual_control(turn_on):
    global manual_pump_on
    manual_pump_on = turn_on
    toggle_water_pump(turn_on)
    if turn_on:
        threading.Timer(3, manual_control_off).start()  # Automatically turn off after 5 seconds

def manual_control_off():
    global manual_pump_on
    manual_pump_on = False
    toggle_water_pump(False)  # Turn off the pump
    # If soil is still dry after 5 seconds, turn the pump back on
    if soil_moisture_level_low():
        threading.Timer(3, manual_control, [True]).start()

def toggle_water_pump(turn_on):
    global pump_on_flag
    if turn_on:
        GPIO.output(RELAY_CONTROL_PIN, 1)
        print("Water pump turned ON")
        pump_on_flag = True
        client.publish("xunyinpi/pump_state", "ON")  # Update pump state in Home Assistant
    else:
        GPIO.output(RELAY_CONTROL_PIN, 0)
        print("Water pump turned OFF")
        pump_on_flag = False
        client.publish("xunyinpi/pump_state", "OFF")  # Update pump state in Home Assistant

try:
    while True:
        # Check moisture level and publish it every 2 seconds
        moisture = round(100 - ((chan.value / 70000) * 100), 2)
        data = {
            "moisture": moisture
        }
        client.publish(publish_topic, json.dumps(data))
        
        # Check if soil moisture is low or pump is manually controlled
        if soil_moisture_level_low() or manual_pump_on:
            toggle_water_pump(True)
            print('Soil is dry, water out!' if soil_moisture_level_low() else 'Pump is manually turned on')
        else:
            toggle_water_pump(False)
            print('Soil is wet, no water out!')

        # Print moisture percentage, raw ADC value, and ADC voltage
        print('Moisture Percentage:', moisture, '%')
        print('Raw ADC Value:', chan.value)
        print('ADC Voltage:', chan.voltage, 'V\n')

        time.sleep(1)  # Wait for 2 seconds
    
except KeyboardInterrupt:
    print("Script terminated by user")
except Exception as e:
    print(f"Unexpected error: {e}")
finally:
    GPIO.cleanup()
    print("GPIO cleanup done")
