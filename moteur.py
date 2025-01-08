import RPi.GPIO as GPIO
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


def gauche() :
    MDV1.start(50)
    MDV2.stop()
    MGV1.stop()
    MGV2.stop()
    print("marche gauche")
    
def avant() :
    MDV1.start(50)
    MDV2.stop()
    MGV1.start(50)
    MGV2.stop()
    print("marche gauche")

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

def stop() :
    MDV1.stop()
    MDV2.stop()
    MGV1.stop()
    MGV2.stop()
    print("Moteurs arretes.")

while True :
    try:    
        a = input("q : gauche, d : droit , z : avant , s : arriere, a : arret")
        if a == "q":
            gauche()
        if a == "d":
            droit()
        if a == "z":
            avant() 
        if a == "s":
            arriere()
        if a == "a":
            stop()
    except Exception as error:
        stop()
        GPIO.cleanup()
        raise error
