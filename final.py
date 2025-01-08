from RPLCD.i2c import CharLCD
import RPi.GPIO as GPIO
import time
import board
import adafruit_dht
import requests
from time import sleep

EN1 = 26
MD_AVANT = 19
MD_ARRIER = 13

MG_AVANT = 6
MG_ARRIER = 5

Motors = [[EN1, MD_AVANT, MD_ARRIER], [EN1, MG_AVANT, MG_ARRIER]]

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(EN1, GPIO.OUT)
GPIO.setup(MD_AVANT, GPIO.OUT)
GPIO.setup(MD_ARRIER, GPIO.OUT)

GPIO.setup(MG_AVANT, GPIO.OUT)
GPIO.setup(MG_ARRIER, GPIO.OUT)

MDV1 = GPIO.PWM(MD_AVANT, 50)
MDV2 = GPIO.PWM(MD_ARRIER, 50)
MGV1 = GPIO.PWM(MG_AVANT, 50)
MGV2 = GPIO.PWM(MG_ARRIER, 50)
sensor = adafruit_dht.DHT11(board.D22)

# Target Spring Boot API endpoint
url2 = "http://192.168.31.213:12500/api/dht/add"
url = "http://192.168.31.213:12500/api/dht/stream"

lcd = CharLCD(i2c_expander='PCF8574', address=0x27, port=1, cols=16, rows=2, dotsize=8)
lcd.clear()

"""
def gauche() :
    MDV1.start(50)
    MDV2.stop()
    MGV1.stop()
    MGV2.stop()
    print("marche gauche")
"""    
def avant() :
    MDV1.start(50)
    MDV2.stop()
    MGV1.start(50)
    MGV2.stop()
    print("marche avant")
"""
def droit() :
    MDV1.stop()
    MDV2.stop()
    MGV1.start(50)
    MGV2.stop()
    print("marche droit")
    
def arriere() :
    MDV1.stop()
    MDV2.start(50)
    MGV1.stop()
    MGV2.start(50)
    print("marche arriere")
"""
def stop() :
    MDV1.stop()
    MDV2.stop()
    MGV1.stop()
    MGV2.stop()
    print("Moteurs arretes.")

# Loop to read sensor data and send it to the API
while True:
    try:
        # Get temperature and humidity from the sensor
        temp = sensor.temperature
        humidity = sensor.humidity
        avant()

        # Create a dictionary with the data to send
        data = {
            "temperature": temp,
            "humidity": humidity
        }
        
        for i in range(10):
            lcd.clear()
            lcd.write_string("Temp: " + str(temp) + "\n\r")
            lcd.write_string("Hum: " + str(humidity))
            time.sleep(0.5)

        # Send the data as a POST request to the Spring Boot API
        response = requests.post(url, json=data)
        response2 = requests.post(url2, json=data)

        print(f"Data sent: {response.status_code} {response2.status_code}")

    except RuntimeError as error:
        print(error.args[0])  # Sensor error
        time.sleep(2.0)
        continue
    except Exception as error:
        sensor.exit()
        stop()
        lcd.clear()
        lcd.exit()
        raise error

    # Wait before reading again
    time.sleep(2.0)

