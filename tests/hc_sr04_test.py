"""
This is a test for the HC-SR04 ultrasonic sensor. 

Sample pinouts: 
    1. VCC — 5V from PIN 2 on PI. 
    2. TRIGGER — GPIO 17, which is PIN 11 on PI. 
    3. ECHO — GPIO 27, which is PIN 13 on PI. 
    4. Ground
"""
import RPi.GPIO as GPIO
import time

# Set GPIO mode
GPIO.setmode(GPIO.BCM)

# Define GPIO pins
TRIG_PIN1 = 17
ECHO_PIN1 = 27

# Setup GPIO pins
GPIO.setup(TRIG_PIN1, GPIO.OUT)
GPIO.setup(ECHO_PIN1, GPIO.IN)

# Define GPIO pins
TRIG_PIN2 = 22
ECHO_PIN2 = 5

# Setup GPIO pins
GPIO.setup(TRIG_PIN2, GPIO.OUT)
GPIO.setup(ECHO_PIN2, GPIO.IN)

# Define GPIO pins
TRIG_PIN3 = 6
ECHO_PIN3 = 26

# Setup GPIO pins
GPIO.setup(TRIG_PIN3, GPIO.OUT)
GPIO.setup(ECHO_PIN3, GPIO.IN)


def distance(TRIG_PIN, ECHO_PIN):
    # Ensure trigger is low
    GPIO.output(TRIG_PIN, False)
    time.sleep(0.5)

    # Send a 10us pulse to trigger
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)

    # Measure the pulse length
    pulse_start = time.time()
    while GPIO.input(ECHO_PIN) == 0:
        pulse_start = time.time()

    pulse_end = time.time()
    while GPIO.input(ECHO_PIN) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start

    # Calculate distance (in cm)
    distance = pulse_duration * 17150
    distance = round(distance, 2)

    return distance

try:
    while True:
        dist1 = distance(TRIG_PIN1, ECHO_PIN1)
        print("Distance1: {} cm".format(dist1))

        dist2 = distance(TRIG_PIN2, ECHO_PIN2)
        print("Distance2: {} cm".format(dist2))

        dist3 = distance(TRIG_PIN3, ECHO_PIN3)
        print("Distance3: {} cm".format(dist3))
        time.sleep(0.1)

except KeyboardInterrupt:
    GPIO.cleanup()

