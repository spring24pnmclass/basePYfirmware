import RPi.GPIO as GPIO
import time
import math
import IMU
import datetime
import sys
from pigps import GPS

GPIO.setmode(GPIO.BCM) 

# Utility classes: 

"""
Class to control L298 Motor Controller with DC Motor and Encoder

Motor Controller: 
IN1 – GPIO 23 (pin 16)
IN2 – GPIO 24 (pin 18)
IN3 – GPIO 10 (pin 21)
IN4 – GPIO 9 (pin 23)
ENA – GPIO 12 (PWM0) (pin 32) 
ENB – GPIO 13 (PWM1) (pin 33) 

"""
class MotorController(): 

    def __init__(self, in1, in2, in3, in4, ena, enb): 
        global pwm_a, pwm_b
        
        self.IN1 = in1 
        self.IN2 = in2 
        self.IN3 = in3 
        self.IN4 = in4 
        self.ENA = ena 
        self.ENB = enb


        # Set up IN1, IN2, IN3, IN4
        if GPIO.gpio_function(self.IN1) != GPIO.OUT: 
            GPIO.setup(self.IN1, GPIO.OUT)

        if GPIO.gpio_function(self.IN2) != GPIO.OUT: 
            GPIO.setup(self.IN2, GPIO.OUT)

        if GPIO.gpio_function(self.IN3) != GPIO.OUT: 
            GPIO.setup(self.IN3, GPIO.OUT)

        if GPIO.gpio_function(self.IN4) != GPIO.OUT: 
            GPIO.setup(self.IN4, GPIO.OUT)

        
        # Set up PWM (ENA, ENB)
        if GPIO.gpio_function(self.ENA) != GPIO.OUT: 
            GPIO.setup(self.ENA, GPIO.OUT)
        
        if GPIO.gpio_function(self.ENB) != GPIO.OUT: 
            GPIO.setup(self.ENB, GPIO.OUT)

        # motor A control
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN1, GPIO.LOW)

        # motor B control 
        GPIO.output(self.IN4, GPIO.LOW) 
        GPIO.output(self.IN3, GPIO.HIGH) 

        pwm_a = GPIO.PWM(self.ENA, 1000)  # 1000 Hz frequency
        pwm_b = GPIO.PWM(self.ENB, 1000)  # 1000 Hz frequency

        pwm_a.start(0)  # Starts with 0% duty cycle
        pwm_b.start(0)  # Starts with 0% duty cycle

    def setSpeed(self, speed):
        pwm_a.start(speed)
        pwm_b.start(speed)

    def stop(self): 
        pwm_a.start(0)
        pwm_b.start(0)

"""
Class to contrl neo6m GPS 

Connected via Micro USB
"""
class neo6m():
    def __init__(self):
        self.gps = GPS()
        self.latitude = 0
        self.longitude = 0
        self.gpstime = 0
        self.altitude = 0
    
    def getLocation(self):
        latitude = self.gps.lat
        longitude = self.gps.lon
        coordinates = [latitude, longitude]
        return coordinates

"""
Class to control the HC-SRO4 ultrasonic sensor. 
@precondition: set mode to GPIO.BCM. 

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
class UltrasonicSensor(): 

    def __init__(self, trigger_pin, echo_pin): 
        self.TRIG = trigger_pin
        self.ECHO = echo_pin
        GPIO.setup(self.TRIG, GPIO.OUT)
        GPIO.setup(self.ECHO, GPIO.IN)

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
            time.sleep(0.0001)  # Add a small delay to prevent busy waiting

        pulse_end = time.time()
        while GPIO.input(self.ECHO) == 1:
            pulse_end = time.time()
            time.sleep(0.0001)  # Add a small delay to prevent busy waiting

        pulse_duration = pulse_end - pulse_start

        # Calculate distance (in cm)
        distance = pulse_duration * 17150
        return round(distance, 2)


"""
Class to control the Berry IMU v3
Connected via I2C protocol
@precondition: set mode to GPIO.BCM. 
"""
class BerryIMU():
    
    def __init__(self):
        
        IMU.detectIMU()     #Detect if BerryIMU is connected.
        if(IMU.BerryIMUversion != 3):
            print(" No BerryIMU found... exiting ")
            sys.exit()
        IMU.initIMU()       #Initialise the accelerometer, gyroscope and compass
        
        self.RAD_TO_DEG = 57.29578
        self.M_PI = 3.14159265358979323846
        self.G_GAIN = 0.070  # [deg/s/LSB] dps for gyro
        self.AA =  0.40      # Complementary filter constant

        # Compass Calibration values (use calibrateBerryIMU.py to get values)
        
        self.magXmin = -1193
        self.magYmin = -1269
        self.magZmin = -2216
        self.magXmax = 788
        self.magYmax = 1177
        self.magZmax = 247
    
    def getHeading(self):
        a = datetime.datetime.now()

        #Read magnetometer values
        MAGx = IMU.readMAGx()
        MAGy = IMU.readMAGy()
        MAGz = IMU.readMAGz()

        #Apply compass calibration
        MAGx -= (self.magXmin + self.magXmax) /2
        MAGy -= (self.magYmin + self.magYmax) /2
        MAGz -= (self.magZmin + self.magZmax) /2

        ##Calculate loop Period(LP). How long between Gyro Reads
        b = datetime.datetime.now() - a
        a = datetime.datetime.now()
        LP = b.microseconds / (1000000 * 1.0)

        #Calculate heading
        heading = 180 * math.atan2(MAGy,MAGx) / self.M_PI

        #Only have our heading between 0 and 360
        if heading < 0:
            heading += 360
        
        return heading

    def getTiltCompensatedHeading(self):
        ACCx = IMU.readACCx()
        ACCy = IMU.readACCy()
        ACCz = IMU.readACCz()

        MAGx = IMU.readMAGx()
        MAGy = IMU.readMAGy()
        MAGz = IMU.readMAGz()
        ################### Tilt compensated heading #########################
        # Normalize accelerometer raw values.
        accXnorm = ACCx / math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
        accYnorm = ACCy / math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)

        # Calculate pitch and roll
        pitch = math.asin(accXnorm)
        roll = -math.asin(accYnorm / math.cos(pitch))

        # Calculate the new tilt compensated values

        # X compensation
        if(IMU.BerryIMUversion == 1 or IMU.BerryIMUversion == 3):            #LSM9DS0 and (LSM6DSL & LIS2MDL)
            magXcomp = MAGx * math.cos(pitch) + MAGz * math.sin(pitch)
        else:                                                                #LSM9DS1
            magXcomp = MAGx * math.cos(pitch) - MAGz * math.sin(pitch)

        # Y compensation
        if(IMU.BerryIMUversion == 1 or IMU.BerryIMUversion == 3):            #LSM9DS0 and (LSM6DSL & LIS2MDL)
            magYcomp = MAGx * math.sin(roll) * math.sin(pitch) + MAGy * math.cos(roll) - MAGz * math.sin(roll) * math.cos(pitch)
        else:                                                                #LSM9DS1
            magYcomp = MAGx * math.sin(roll) * math.sin(pitch)+MAGy*math.cos(roll)+MAGz*math.sin(roll)*math.cos(pitch)

        # Calculate tilt compensated heading
        tiltCompensatedHeading = 180 * math.atan2(magYcomp, magXcomp) / self.M_PI

        if tiltCompensatedHeading < 0:
            tiltCompensatedHeading += 360

        return tiltCompensatedHeading        

class TogoBot():

    def __init__(self):
        # Motor Controller and DC Motors w/ Encoders
        self.motors = MotorController(in1=23, in2=24, in3=10, in4=9, ena=12, enb=13)
        
        # Ultrasonic Sensors
        self.ultrasonic1 = UltrasonicSensor(trigger_pin=17, echo_pin=27)
        self.ultrasonic2 = UltrasonicSensor(trigger_pin=22, echo_pin=5)
        self.ultrasonic3 = UltrasonicSensor(trigger_pin=6, echo_pin=26)

        # NEO6M GPS
        self.gps = neo6m()

        # Berry IMU
        self.berryimu = BerryIMU()

    def runBot(self):

        try:
            while True: 
                dist1 = self.ultrasonic1.getDistance()
                time.sleep(0.01)
                dist2 = self.ultrasonic2.getDistance()
                time.sleep(0.01)
                dist3 = self.ultrasonic3.getDistance()
                time.sleep(0.01)

                if dist1 < 100 or dist2 < 100 or dist3 < 100:
                    self.motors.stop()
                else:
                    self.motors.setSpeed(50)

                time.sleep(0.01)
        
        except KeyboardInterrupt:
            # Clean up GPIO on Ctrl+C exit
            GPIO.cleanup()

    def getLocation(self):
        return self.gps.getLocation()

    def getCompassDirection(self):
        return self.berryimu.getTiltCompensatedHeading()


# initialize TogoBot and have robot move 
robot = TogoBot()
robot.runBot()