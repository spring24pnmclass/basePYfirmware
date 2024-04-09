import RPi.GPIO as GPIO
import time

# Set the GPIO numbering mode
# GPIO.setmode(GPIO.BOARD)
GPIO.setmode(GPIO.BCM) 

# Define the GPIO pins
iopin = 23

GPIO.setup(iopin, GPIO.OUT)

def testHigh():
    GPIO.output(iopin, GPIO.HIGH)

def testLow():
    GPIO.output(iopin, GPIO.LOW)

try:
    while True:
        # Rotate motor clockwise for 2 seconds
        print("low")
        testLow()
        time.sleep(2)

        # Rotate motor counterclockwise for 2 seconds
        print("high")
        testHigh()
        time.sleep(2)

except KeyboardInterrupt:
    # Clean up GPIO on Ctrl+C exit
    GPIO.cleanup()

