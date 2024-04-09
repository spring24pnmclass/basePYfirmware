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
# GPIO.setmode(GPIO.BOARD)
GPIO.setmode(GPIO.BCM) 

# Define the GPIO pins
IN1 = 23
IN2 = 24

# Check if the channel is already in use: 
if GPIO.gpio_function(IN2) != GPIO.OUT: 
    print("IN2 is being setup") 
    GPIO.setup(IN2, GPIO.OUT)

if GPIO.gpio_function(IN1) != GPIO.OUT: 
    print("IN1 is being setup") 
    GPIO.setup(IN1, GPIO.OUT)

# Function to set motor direction
def set_motor_direction(clockwise):
    if clockwise:
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN1, GPIO.LOW)
    else:
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN1, GPIO.HIGH)

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

