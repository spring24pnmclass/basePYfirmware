"""
Code to test the L298 motor controller. 

The following are the pins to a double motor controller configuration: 
    1. GPIO 23 — IN 1
    2. GPIO 24 — IN 2    
    3. GPIO 25 - IN 3
    4. GPIO 7  - IN 4

    5. GPIO 12 (PWM0) -- PIN 32 -- ENA 
    6. GPIO 13 (PWM1) -- PIN 33 -- ENB
"""

import RPi.GPIO as GPIO
import time

# Set the GPIO numbering mode
# GPIO.setmode(GPIO.BOARD)
GPIO.setmode(GPIO.BCM) 

# Define the GPIO pins for H-Bridge control
IN1 = 23
IN2 = 24

IN3 = 10
IN4 = 9

# Define GPIO for PWM speed control 
ENA = 12
ENB = 13

# Check if the channel is already in use: 
if GPIO.gpio_function(IN1) != GPIO.OUT: 
    print("IN1 is being setup") 
    GPIO.setup(IN1, GPIO.OUT)

if GPIO.gpio_function(IN2) != GPIO.OUT: 
    print("IN2 is being setup") 
    GPIO.setup(IN2, GPIO.OUT)

if GPIO.gpio_function(IN3) != GPIO.OUT: 
    print("IN3 is being setup") 
    GPIO.setup(IN3, GPIO.OUT)

if GPIO.gpio_function(IN4) != GPIO.OUT: 
    print("IN4 is being setup") 
    GPIO.setup(IN4, GPIO.OUT)

if GPIO.gpio_function(ENA) != GPIO.OUT: 
    print("ENA is being setup") 
    GPIO.setup(ENA, GPIO.OUT)

if GPIO.gpio_function(ENB) != GPIO.OUT: 
    print("ENB is being setup") 
    GPIO.setup(ENB, GPIO.OUT)

# GPIO.setup(ENA, GPIO.OUT)
# GPIO.setup(ENB, GPIO.OUT)
pwm_a = GPIO.PWM(ENA, 1000)  # 1000 Hz frequency
pwm_b = GPIO.PWM(ENB, 1000)  # 1000 Hz frequency

pwm_a.start(0)  # Starts with 0% duty cycle
pwm_b.start(0)  # Starts with 0% duty cycle

# Function to set motor direction
def set_motor_direction(clockwise):
    if clockwise:
        # motor A control
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN1, GPIO.LOW)

        # motor B control 
        GPIO.output(IN4, GPIO.LOW) 
        GPIO.output(IN3, GPIO.HIGH) 
    else: 
        # motor A control 
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN1, GPIO.HIGH)

        # motor B control 
        GPIO.output(IN4, GPIO.HIGH) 
        GPIO.output(IN3, GPIO.LOW) 

def motor_direction_test(): 
    # Rotate motor clockwise for 2 seconds
    print("Rotating both motors clockwise")
    set_motor_direction(clockwise=True)
    time.sleep(3)

    # Rotate motor counterclockwise for 2 seconds
    print("Rotating both motors counterclockwise")
    set_motor_direction(clockwise=False)
    time.sleep(3)

def motor_speed_test(): 
    print("setup")
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN1, GPIO.HIGH)

    GPIO.output(IN4, GPIO.LOW) 
    GPIO.output(IN3, GPIO.HIGH) 

    speeds = [2, 25, 50, 75, 100]
    index = 0 
    while True: 
        print("setting speed to ", speeds[index])
        pwm_a.start(speeds[index])
        pwm_b.start(speeds[index])

        time.sleep(3)
        if(index == len(speeds) - 1): 
            index = 0
        else: 
            index+=1

try:

    set_motor_direction(clockwise=True)

    pwm_a.start(20)
    pwm_b.start(20)
    print("set 20") 

    time.sleep(0.5) 

    pwm_a.start(40)
    pwm_b.start(40)
    print("set 40") 

    time.sleep(0.5) 

    pwm_a.start(60)
    pwm_b.start(60)
    print("set 60") 
    
    time.sleep(0.5) 
    
    pwm_a.start(80)
    pwm_b.start(80)
    print("set 80") 

    time.sleep(0.5) 

    pwm_a.start(100)
    pwm_b.start(100)
    print("set 100") 

    while True: 
        pass

    GPIO.cleanup()
except KeyboardInterrupt:
    # Clean up GPIO on Ctrl+C exit
    GPIO.cleanup()
