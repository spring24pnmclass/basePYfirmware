import RPi.GPIO as GPIO
import time

TICKS_PER_REVOLUTION = 64

# Set the GPIO numbering mode
# GPIO.setmode(GPIO.BOARD)
GPIO.setmode(GPIO.BCM) 

# Initialize variables for speed and direction
speed_in_pulse_count = 0
direction = 1  # 1 for clockwise, -1 for counterclockwise

# Initialize time variables for speed calculation
prev_time = time.time()
prev_pulse_count = 0

# Define the GPIO pins for H-Bridge control
IN1 = 23
IN2 = 24

IN3 = 10
IN4 = 9

# Define GPIO for PWM speed control 
ENA = 12
ENB = 13

# Define encoder values: 
M1_ENC_A = 25 
M1_ENC_B = 8

GPIO.setup(M1_ENC_A, GPIO.IN)
GPIO.setup(M1_ENC_B, GPIO.IN)

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

GPIO.setup(ENA, GPIO.OUT)
# GPIO.setup(ENB, GPIO.OUT)
pwm_a = GPIO.PWM(ENA, 1000)  # 1000 Hz frequency
# pwm_b = GPIO.PWM(ENB, 1000)  # 1000 Hz frequency

pwm_a.start(50)  # Starts with 0% duty cycle
# pwm_b.start(0)  # Starts with 0% duty cycle

def motor_direction(clockwise): 
    if clockwise:
        # motor A control
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN1, GPIO.LOW)

        # motor B control 
        # GPIO.output(IN4, GPIO.HIGH) 
        # GPIO.output(IN3, GPIO.LOW) 
    else: 
        # motor A control 
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN1, GPIO.HIGH)

        # motor B control 
        # GPIO.output(IN4, GPIO.LOW) 
        # GPIO.output(IN3, GPIO.HIGH) 

def motor_speed_test(): 
    print("setup")
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN1, GPIO.HIGH)

    # GPIO.output(IN4, GPIO.LOW) 
    # GPIO.output(IN3, GPIO.HIGH) 

    speeds = [2, 25, 50, 75, 100]
    index = 0 
    while True: 
        # print("setting speed to ", speeds[index])
        pwm_a.start(speeds[index])
        # pwm_b.start(speeds[index])

        time.sleep(3)
        if(index == len(speeds) - 1): 
            index = 0
        else: 
            index+=1

def pulses_to_rotation(pulse_count):
    return pulse_count / TICKS_PER_REVOLUTION

"""
Looks like this setups of a function asynchronously. 
"""
def motor_1_enc_a_callback(channel): 
    global prev_pulse_count, prev_time, speed_in_pulse_count, direction

    # Measure time between pulses to calculate speed
    curr_time = time.time()
    pulse_count = prev_pulse_count + 1
    pulse_delta = pulse_count - prev_pulse_count
    time_delta = curr_time - prev_time
    speed_in_pulse_count = pulse_delta / time_delta

    # Update time and pulse count for next calculation
    prev_time = curr_time
    prev_pulse_count = pulse_count

    # Determine direction based on channel B state
    if GPIO.input(M1_ENC_B):
        direction = 1  # Clockwise
        # print("Clockwise")
    else:
        direction = -1  # Counterclockwise
        # print("Counterclockwise")
    
    print("Speed: {:.2f} rotations/second, Direction: {}".format(pulses_to_rotation(speed_in_pulse_count), "Clockwise" if direction == 1 else "Counterclockwise"))

GPIO.add_event_detect(M1_ENC_A, GPIO.RISING, callback=motor_1_enc_a_callback)

try:
    while True: 
        pwm_a.start(50)
        motor_direction(clockwise=True)
        time.sleep(3)

        pwm_a.start(0)
        time.sleep(0.1)

        motor_direction(clockwise=False)
        pwm_a.start(50)
        time.sleep(3)

        pwm_a.start(0)
        time.sleep(0.1)

except KeyboardInterrupt:
    # Clean up GPIO on Ctrl+C exit
    GPIO.cleanup()