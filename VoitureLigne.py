from RPLCD.i2c import CharLCD
import RPi.GPIO as GPIO
import time
import board
import adafruit_dht
import requests

# Motor pins
ENA = 12
ENB = 13
IN1 = 19
IN2 = 26
IN3 = 6
IN4 = 5

# RADAR pins
ECHO_A = 17
TRIG_A = 18
ECHO_D = 27
TRIG_D = 22
ECHO_G = 23
TRIG_G = 24

# IR Sensor pins (assuming two IR sensors for line detection)
IR_LEFT_PIN = 25  # Left IR sensor
IR_RIGHT_PIN = 8  # Right IR sensor

# DHT11
sensor = adafruit_dht.DHT11(board.D4)

# LCD
lcd = CharLCD(i2c_expander='PCF8574', address=0x27, port=1, cols=16, rows=2, dotsize=8)
lcd.clear()

# Group motor and radar pins
motors = [[ENA, IN1, IN2], [ENB, IN3, IN4]]
radars = [[ECHO_A, TRIG_A], [ECHO_D, TRIG_D], [ECHO_G, TRIG_G]]

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Setup motor pins
for motor in motors:
    GPIO.setup(motor[0], GPIO.OUT)  # ENA and ENB
    GPIO.setup(motor[1], GPIO.OUT)  # IN1, IN3
    GPIO.setup(motor[2], GPIO.OUT)  # IN2, IN4

# Setup radar pins
for radar in radars:
    GPIO.setup(radar[0], GPIO.IN)  # ECHO pins
    GPIO.setup(radar[1], GPIO.OUT)  # TRIG pins

# Setup IR sensor pins
GPIO.setup(IR_LEFT_PIN, GPIO.IN)  # Left IR sensor
GPIO.setup(IR_RIGHT_PIN, GPIO.IN)  # Right IR sensor

# Initialize PWM for motors
M0_PWM = GPIO.PWM(motors[0][0], 50)  # 50 Hz
M1_PWM = GPIO.PWM(motors[1][0], 50)  # 50 Hz

M0_PWM.start(0)
M1_PWM.start(0)

def measure_distance(echo, trig):
    """Measure the distance in cm using ultrasonic sensor."""
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    start_time = time.time()
    stop_time = time.time()

    while GPIO.input(echo) == 0:
        start_time = time.time()

    while GPIO.input(echo) == 1:
        stop_time = time.time()

    elapsed_time = stop_time - start_time
    distance = (elapsed_time * 34300) / 2  # Speed of sound = 34300 cm/s
    return distance

def marche_avant(M0_PWM, M1_PWM, motors):
    for m in motors:
        GPIO.output(m[1], 1)
        GPIO.output(m[2], 0)
    M0_PWM.ChangeDutyCycle(100)
    M1_PWM.ChangeDutyCycle(100)

def marche_arriere(M0_PWM, M1_PWM, motors):
    for m in motors:
        GPIO.output(m[2], 1)
        GPIO.output(m[1], 0)
    M0_PWM.ChangeDutyCycle(100)
    M1_PWM.ChangeDutyCycle(100)

def devier_gauche(M0_PWM, M1_PWM, motors):
    marche_avant(M0_PWM, M1_PWM, motors)
    M1_PWM.ChangeDutyCycle(50)
    time.sleep(0.5)
    M1_PWM.ChangeDutyCycle(100)

def devier_droite(M0_PWM, M1_PWM, motors):
    marche_avant(M0_PWM, M1_PWM, motors)
    M0_PWM.ChangeDutyCycle(50)
    time.sleep(0.5)
    M0_PWM.ChangeDutyCycle(100)

def stop_motors(M0_PWM, M1_PWM, motors):
    for m in motors:
        GPIO.output(m[1], 0)
        GPIO.output(m[2], 0)
    M0_PWM.ChangeDutyCycle(0)
    M1_PWM.ChangeDutyCycle(0)

def line_follow():
    """Line-following logic using IR sensors."""
    left_ir = GPIO.input(IR_LEFT_PIN)
    right_ir = GPIO.input(IR_RIGHT_PIN)

    if left_ir == 0 and right_ir == 0:
        # Both sensors are on the line, move straight ahead
        marche_avant(M0_PWM, M1_PWM, motors)
    elif left_ir == 0 and right_ir == 1:
        # Left sensor on the line, turn right
        devier_droite(M0_PWM, M1_PWM, motors)
    elif left_ir == 1 and right_ir == 0:
        # Right sensor on the line, turn left
        devier_gauche(M0_PWM, M1_PWM, motors)
    else:
        # Both sensors off the line, stop
        stop_motors(M0_PWM, M1_PWM, motors)

def adjust_direction(left_distance, right_distance):
    if left_distance < 10 and right_distance > 10:  # Example condition
        devier_droite(M0_PWM, M1_PWM, motors)  # Turn right
    elif right_distance < 10 and left_distance > 10:  # Example condition
        devier_gauche(M0_PWM, M1_PWM, motors)  # Turn left

while True:
    try:
        # Get temperature and humidity from the sensor
        temp = sensor.temperature
        humidity = sensor.humidity
        
        for i in range(10):
            lcd.clear()
            lcd.write_string("Temp: " + str(temp) + "\n\r")
            lcd.write_string("Hum: " + str(humidity))
            time.sleep(0.5)

        # Measure the front distance
        front_distance = measure_distance(radars[0][0],radars[0][1])

        # If there's an obstacle too close in front, stop the robot
        if front_distance < 20:  # 10 cm threshold
            stop_motors(M0_PWM, M1_PWM, motors)
            lcd.clear()
            lcd.write_string("Obstacle detected!")
            time.sleep(1)
            lcd.clear()
            lcd.write_string("Stopping robot...")
            time.sleep(1)
            continue  # Skip the rest of the loop until user decides to move again

        # Follow the line using IR sensors
        line_follow()

        right_distance = measure_distance(radars[1][0],radars[1][1])
        left_distance = measure_distance(radars[2][0],radars[2][1])
        
        # Call the adjust_direction function to avoid obstacles
        adjust_direction(left_distance, right_distance)

    except RuntimeError as error:
        print(error.args[0])  # Sensor error
        time.sleep(2.0)
        continue
    except Exception as error:
        sensor.exit()
        stop_motors(M0_PWM, M1_PWM, motors)
        M0_PWM.stop()
        M1_PWM.stop()
        lcd.clear()
        lcd.exit()
        raise error

    # Wait before reading again
    time.sleep(2.0)

