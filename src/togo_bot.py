import RPi.GPIO as GPIO
import time
import math
import IMU
import datetime
import os
import sys

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
        self.magXmin = -2680
        self.magYmin = -2393
        self.magZmin = -3572
        self.magXmax = 2346
        self.magYmax = 2623
        self.magZmax = 1411

    
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
        ###################Tilt compensated heading#########################
        #Normalize accelerometer raw values.
        accXnorm = ACCx / math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
        accYnorm = ACCy / math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)

        #Calculate pitch and roll
        pitch = math.asin(accXnorm)
        roll = -math.asin(accYnorm / math.cos(pitch))

        #Calculate the new tilt compensated values

        #X compensation
        if(IMU.BerryIMUversion == 1 or IMU.BerryIMUversion == 3):            #LSM9DS0 and (LSM6DSL & LIS2MDL)
            magXcomp = MAGx * math.cos(pitch) + MAGz * math.sin(pitch)
        else:                                                                #LSM9DS1
            magXcomp = MAGx * math.cos(pitch) - MAGz * math.sin(pitch)

        #Y compensation
        if(IMU.BerryIMUversion == 1 or IMU.BerryIMUversion == 3):            #LSM9DS0 and (LSM6DSL & LIS2MDL)
            magYcomp = MAGx * math.sin(roll) * math.sin(pitch) + MAGy * math.cos(roll) - MAGz * math.sin(roll) * math.cos(pitch)
        else:                                                                #LSM9DS1
            magYcomp = MAGx * math.sin(roll) * math.sin(pitch)+MAGy*math.cos(roll)+MAGz*math.sin(roll)*math.cos(pitch)

        #Calculate tilt compensated heading
        tiltCompensatedHeading = 180 * math.atan2(magYcomp, magXcomp) / self.M_PI

        if tiltCompensatedHeading < 0:
            tiltCompensatedHeading += 360

        return tiltCompensatedHeading        

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

imu = BerryIMU()

while True: 
    tilt = imu.getTiltCompensatedHeading(imu)
    heading = imu.getHeading(imu)

    print("\t# HEADING %5.2f  tiltCompensatedHeading %5.2f #" % (heading, tilt))





