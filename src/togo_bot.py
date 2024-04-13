import RPi.GPIO as GPIO
import time

# Utility classes: 
class MotorController(): 

    def __init__(self, in1, in2, in3, in4, ena, enb): 
        self.IN1 = in1 
        self.IN2 = in2 
        self.IN3 = in3 
        self.IN4 = in4 
        self.ENA = ena 
        self.ENB = enb

"""
Class to control the HC-SRO4 ultrasonic sensor. 
@precondition: set mode to GPIO.BCM. 
"""
class UltrasonicSensor(): 

    def __init__(self, trigger_pin, echo_pin): 
        self.TRIG = trigger_pin
        self.ECHO = echo_pin
        GPIO.setmode(self.TRIG, GPIO.OUT)
        GPIO.setmode(self.ECHO, GPIO.IN)

    def getDistance(self): 
        # Ensure trigger is low
        GPIO.output(self.TRIG, False)
        time.sleep(0.001)

        # Send a 10us pulse to trigger
        GPIO.output(self.TRIG, True)
        time.sleep(0.00001)
        GPIO.output(self.TRIG, False)

        # Measure the pulse length
        pulse_start = time.time()
        while GPIO.input(self.ECHO) == 0:
            pulse_start = time.time()

        pulse_end = time.time()
        while GPIO.input(self.ECHO) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start

        # Calculate distance (in cm)
        distance = pulse_duration * 17150
        return round(distance, 2)

"""
Sensor 1: 
    TRIGGER – GPIO 17 (pin 11)
    ECHO – GPIO 27 (pin 13)
Sensor 2: 
    TRIGGER – GPIO 22 (pin 15)
    ECHO – GPIO 5 (pin 29)
Sensor 3:
    TRIGGER – GPIO 6 (pin 31)
    ECHO – GPIO 26 (pin 37)

"""
class TogoBot(): 

    def __init__(self): 
        GPIO.setmode(GPIO.BCM)

        # Init sensors: 
        self.sensor1 = UltrasonicSensor(trigger_pin=17, echo_pin=27)
        self.sensor2 = UltrasonicSensor(trigger_pin=22, echo_pin=5)
        self.sensor3 = UltrasonicSensor(trigger_pin=6, echo_pin=26)