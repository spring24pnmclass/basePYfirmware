"""
Code to test the L298 motor controller. 

The following are the pins to a single motor controller configuration: 
    1. GPIO 23 — IN 4
    2. GPIO 24 — IN 3    

    TODO : Finish adding a ENA and ENB to control the speed of the motor. Currently, not using that pin allows the motor to spin at full speed. 
"""

import RPi.GPIO as GPIO
import time

# Set the GPIO numbering mode
GPIO.setmode(GPIO.BOARD)
# GPIO.setmode(GPIO.BCM) 

# Define the GPIO pins
IN4 = 23
IN3 = 24

# Check if the channel is already in use: 
if GPIO.gpio_function(IN4) != GPIO.OUT: 
    GPIO.setup(IN4, GPIO.OUT)

if GPIO.gpio_function(IN3) != GPIO.OUT: 
    GPIO.setup(IN3, GPIO.OUT)

# Function to set motor direction
def set_motor_direction(clockwise):
    if clockwise:
        GPIO.output(IN4, GPIO.HIGH)
        GPIO.output(IN3, GPIO.LOW)
    else:
        GPIO.output(IN4, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)

try:
    while True:
        # Rotate motor clockwise for 2 seconds
        print("Rotating motor clockwise")
        set_motor_direction(clockwise=True)
        time.sleep(2)

        # Rotate motor counterclockwise for 2 seconds
        print("Rotating motor counterclockwise")
        set_motor_direction(clockwise=False)
        time.sleep(2)

except KeyboardInterrupt:
    # Clean up GPIO on Ctrl+C exit
    GPIO.cleanup()

