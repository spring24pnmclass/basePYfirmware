"""
Code to test the SG-90 9g servo motor. 

The following are the sample pins that were used: 
    1. VCC (red wire) — 5V from PIN 2 on PI.
    2. PWM (orange wire) — GPIO 22 which is PIN 15 on PI.
    3. Ground
"""

import RPi.GPIO as GPIO
import time

# Set GPIO mode
GPIO.setmode(GPIO.BCM)

# Define GPIO pins
PWM_PIN = 22

# Setup PWM pin
GPIO.setup(PWM_PIN, GPIO.OUT)
pwm = GPIO.PWM(PWM_PIN, 50)  # 50 Hz frequency for SG-90 servo

# Start PWM with 0% duty cycle (servo at 0 degrees)
pwm.start(0)

def set_angle(angle):
    duty = angle / 18 + 2
    GPIO.output(PWM_PIN, True)
    pwm.ChangeDutyCycle(duty)
    time.sleep(1)
    GPIO.output(PWM_PIN, False)
    pwm.ChangeDutyCycle(0)

try:
    while True:
        # Test different angles
        print("turn 0") 
        set_angle(0)   # 0 degrees
        time.sleep(2)
        print("turn 90")
        set_angle(90)  # 90 degrees
        time.sleep(2)
        print("turn 180") 
        set_angle(180) # 180 degrees
        time.sleep(2)

except KeyboardInterrupt:
    pwm.stop()
    GPIO.cleanup()

