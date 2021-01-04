#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from time import sleep
from ev3dev.ev3 import *
from math import *
import threading
from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_D, SpeedPercent
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.led import Leds

# ---------------------------------
# Global constants for this file.

# In seconds
timeDelta = 0.02
# In meters
wheelDiameter = 5.5/100
# In meters
wheelCircumference = 17.3/100
# In meters
vehicleWidth = 12.7/100


# Motors
motorLeft = LargeMotor(OUTPUT_D)
motorRight = LargeMotor(OUTPUT_C)


# -------------------------------

# Connect gyro
gy = GyroSensor()

# Put the gyro sensor into ANGLE mode.
gy.mode = 'GYRO-ANG'


assert motorRight.connected
assert motorLeft.connected


###########
# Functions for measurement
# These functions will run in their own thread.
###########


# currentTachoReading: int. Degrees of rotation per second for a wheel in the current measurement.
# previousTachoReading: int. Degrees of rotation per second for a wheel in the previous measurement.
# returns: float. Speed of the wheel in m/s
def wheelVelocity(currentTachoReading, previousTachoReading):
    return (wheelDiameter/2)*((currentTachoReading - previousTachoReading)/timeDelta)*((2*pi)/360)


# currentTachoReadingLeft: int. Degrees of rotation  per second for left wheel in the current measurement.
# previousTachoReadingLeft: int. Degrees of rotation per second of left wheel  in the previous measurement.
# currentTachoReadingLeft: int. Degrees of rotation  per second for right wheel in the current measurement.
# previousTachoReadingLeft: int. Degrees of rotation per second of right wheel  in the previous measurement.
# returns: float. Angular speed velocity of vehicle, in radians/sec
def angularVelocity(currentTachoReadingLeft, previousTachoReadingLeft, currentTachoReadingRight, previousTachoReadingRight):
    return (wheelVelocity(currentTachoReadingRight, previousTachoReadingRight) - wheelVelocity(currentTachoReadingLeft, previousTachoReadingLeft))/vehicleWidth


# currentTachoReadingLeft: int. Degrees of rotation  per second for left wheel in the current measurement.
# previousTachoReadingLeft: int. Degrees of rotation per second of left wheel  in the previous measurement.
# currentTachoReadingLeft: int. Degrees of rotation  per second for right wheel in the current measurement.
# previousTachoReadingLeft: int. Degrees of rotation per second of right wheel  in the previous measurement.
# returns: float. Speed of the vehicle, essentially, an average for the speeds of the two wheels. in m/s
def vehicleVelocity(currentTachoReadingLeft, previousTachoReadingLeft, currentTachoReadingRight, previousTachoReadingRight):
    return (wheelVelocity(currentTachoReadingRight, previousTachoReadingRight) + wheelVelocity(currentTachoReadingLeft, previousTachoReadingLeft))/2


# currentTachoReadingLeft: int. Degrees of rotation  per second for left wheel in the current measurement.
# previousTachoReadingLeft: int. Degrees of rotation per second of left wheel  in the previous measurement.
# currentTachoReadingLeft: int. Degrees of rotation  per second for right wheel in the current measurement.
# previousTachoReadingLeft: int. Degrees of rotation per second of right wheel  in the previous measurement.
# returns: float. The radius of a circle centered in the ICC of the vehicle in meters
# TODO what happens angularVelocity is 0? (vehicle is moving in a straight line and both wheels at the same speed) should this be 0?
#     according to http://www.cs.columbia.edu/~allen/F15/NOTES/icckinematics.pdf (page 2) it should be infinite. What do you think?
def radiusOfRotation(currentTachoReadingLeft, previousTachoReadingLeft, currentTachoReadingRight, previousTachoReadingRight):
    return vehicleVelocity(currentTachoReadingLeft, previousTachoReadingLeft, currentTachoReadingRight, previousTachoReadingRight)/angularVelociy(currentTachoReadingLeft, previousTachoReadingLeft, currentTachoReadingRight, previousTachoReadingRight)

# angularVelocityTimeSeries: list<float>. Contains the discrete measurements for the angular velocity from 0 to (len(angularVelocityTimeSeries)-1)*deltaTime seconds
# returns: float. angle of rotation in radians


def theta(angule):
    return angule*timeDelta

# angularVelocityTimeSeries: list<float>. Contains the discrete measurements for the angular velocity from 0 to (len(angularVelocityTimeSeries)-1)*deltaTime seconds
# velocityTimeSeries: list<float>. Contains the discrete measurements for the linear velocity from 0 to (len(velocityTimeSeries)-1)*deltaTime seconds
# returns: float. the x coordinate measured from the point of origin. In meters.
def positionX(velocity, angle):
    return velocity*cos(angle)*timeDelta

# angularVelocityTimeSeries: list<float>. Contains the discrete measurements for the angular velocity from 0 to (len(angularVelocityTimeSeries)-1)*deltaTime seconds
# velocityTimeSeries: list<float>. Contains the discrete measurements for the linear velocity from 0 to (len(velocityTimeSeries)-1)*deltaTime seconds
# returns: float. the x coordinate measured from the point of origin. In meters.
def positionY(velocity, angle):
    return velocity*sin(angle)*timeDelta

###########
# End of Functions for measurement
###########



# commands correspond to [left-motor-speed, right-motor-speed, time duration (s)]
#commands = [[80, 60, 2], [60, 60, 1], [-50, 80, 2]]
commands = [[20,40,2]]

# Start measurement thread. This is a daemon thread which will terminate once
# The main program terminates.
#measuringThread = threading.Thread(target=measure, daemon=True)
#measuringThread.start()


# Run the commands given in the variable
# TODO instead of multithreading we could also consider refreshing this loop
#     every timeDelta seconds, and then figuring out when enough seconds have passed
#     so that we can move to the next instruction.
currentTachoReadingLeft = motorLeft.position
currentTachoReadingRight = motorRight.position
posX=0
posY=0
angle=0



for command in commands:
    runTime = 0
    leftSpeed = (command[0]*0.01)*900
    rightSpeed = (command[1]*0.01)*900
    motorRight.run_timed(speed_sp=rightSpeed, time_sp=command[2]*1000)
    motorLeft.run_timed(speed_sp=leftSpeed, time_sp=command[2]*1000)
    while (runTime <= command[2]):
        wTimeSeries = []
        vTimeSeries = []
        angleTimeSeries = []
        previousTachoReadingLeft = currentTachoReadingLeft
        previousTachoReadingRight = currentTachoReadingRight
        sleep(timeDelta)
        runTime = runTime+timeDelta
        currentTachoReadingLeft = motorLeft.position
        currentTachoReadingRight = motorRight.position

        currentAngularVelocity = angularVelocity(currentTachoReadingLeft, previousTachoReadingLeft,
                                            currentTachoReadingRight, previousTachoReadingRight)
        currentVehicleVelocity = vehicleVelocity(currentTachoReadingLeft, previousTachoReadingLeft,
                                            currentTachoReadingRight, previousTachoReadingRight)

        angle = angle + theta(currentAngularVelocity)
        posX = posX + positionX(currentVehicleVelocity, angle)
        posY = posY + positionY(currentVehicleVelocity, angle)
    sleep(0.5)



print("Theta(t): " + str(((angle*360)/(2*pi)) % 360) + "\n")
print("PosX(t): %.2f cm\n" % (posX*100))
print("PosY(t): %.2f cm\n" % (posY*100))
motorRight.stop()
motorLeft.stop()
