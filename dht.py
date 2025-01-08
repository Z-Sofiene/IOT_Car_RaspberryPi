import time
import board
import adafruit_dht
import requests

# Set up DHT sensor
sensor = adafruit_dht.DHT11(board.D22)

# Target Spring Boot API endpoint
url2 = "http://192.168.31.213:12500/api/dht/add"
url = "http://192.168.31.213:12500/api/dht/stream"
# Loop to read sensor data and send it to the API
while True:
    try:
        # Get temperature and humidity from the sensor
        temp = sensor.temperature
        humidity = sensor.humidity

        # Create a dictionary with the data to send
        data = {
            "temperature": temp,
            "humidity": humidity
        }

        # Send the data as a POST request to the Spring Boot API
        response = requests.post(url, json=data)
        response = requests.post(url2, json=data)

        print(f"Data sent: {response.status_code} {response.text}")

    except RuntimeError as error:
        print(error.args[0])  # Sensor error
        time.sleep(2.0)
        continue
    except Exception as error:
        sensor.exit()
        raise error

    # Wait before reading again
    time.sleep(2.0)
