from time import sleep
from ev3dev.ev3 import *
from math import *
import threading
from ev3dev2.motor import LargeMotor, OUTPUT_C, OUTPUT_D, SpeedPercent
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.led import Leds

# ---------------------------------
# Global constants for this file.

class DDrive():
    def __init__(self, base, joint):
            self.motorLeft = base
            self.motorRight = joint
            self.currentTachoReadingLeft = self.motorLeft.position
            self.currentTachoReadingRight = self.motorRight.position
            self.previousTachoReadingLeft = self.currentTachoReadingLeft
            self.previousTachoReadingRight = self.currentTachoReadingRight
            self.prev_x = 0
            self.prev_y = 0
            self.angle = 0
            self.current_x = 0
            self.current_y = 0
            self.eps = 2 * 10e-7

            # In seconds
            self.timeDelta = 0.02
            # In meters
            self.wheelDiameter = 5.5/100
            # In meters
            self.wheelCircumference = 17.3/100
    # In meters
            self.vehicleWidth = 12.7/100

    # currentTachoReading: int. Degrees of rotation per second for a wheel in the current measurement.
    # previousTachoReading: int. Degrees of rotation per second for a wheel in the previous measurement.
    # returns: float. Speed of the wheel in m/s
    def wheelVelocity(self, currentTachoReading, previousTachoReading):
        return (self.wheelDiameter/2)*((currentTachoReading - previousTachoReading)/self.timeDelta)*((2*pi)/360)


    # currentTachoReadingLeft: int. Degrees of rotation  per second for left wheel in the current measurement.
    # previousTachoReadingLeft: int. Degrees of rotation per second of left wheel  in the previous measurement.
    # currentTachoReadingLeft: int. Degrees of rotation  per second for right wheel in the current measurement.
    # previousTachoReadingLeft: int. Degrees of rotation per second of right wheel  in the previous measurement.
    # returns: float. Angular speed velocity of vehicle, in radians/sec
    def angularVelocity(self):
        return (self.wheelVelocity(self.currentTachoReadingRight, self.previousTachoReadingRight) - self.wheelVelocity(self.currentTachoReadingLeft, self.previousTachoReadingLeft))/self.vehicleWidth


    # currentTachoReadingLeft: int. Degrees of rotation  per second for left wheel in the current measurement.
    # previousTachoReadingLeft: int. Degrees of rotation per second of left wheel  in the previous measurement.
    # currentTachoReadingLeft: int. Degrees of rotation  per second for right wheel in the current measurement.
    # previousTachoReadingLeft: int. Degrees of rotation per second of right wheel  in the previous measurement.
    # returns: float. Speed of the vehicle, essentially, an average for the speeds of the two wheels. in m/s
    def vehicleVelocity(self):
        return (self.wheelVelocity(self.currentTachoReadingRight, self.previousTachoReadingRight) + self.wheelVelocity(self.currentTachoReadingLeft, self.previousTachoReadingLeft))/2


    # currentTachoReadingLeft: int. Degrees of rotation  per second for left wheel in the current measurement.
    # previousTachoReadingLeft: int. Degrees of rotation per second of left wheel  in the previous measurement.
    # currentTachoReadingLeft: int. Degrees of rotation  per second for right wheel in the current measurement.
    # previousTachoReadingLeft: int. Degrees of rotation per second of right wheel  in the previous measurement.
    # returns: float. The radius of a circle centered in the ICC of the vehicle in meters
    # TODO what happens angularVelocity is 0? (vehicle is moving in a straight line and both wheels at the same speed) should this be 0?
    #     according to http://www.cs.columbia.edu/~allen/F15/NOTES/icckinematics.pdf (page 2) it should be infinite. What do you think?
    def radiusOfRotation(self):
        return self.vehicleVelocity()/self.angularVelociy()

    # angularVelocityTimeSeries: list<float>. Contains the discrete measurements for the angular velocity from 0 to (len(angularVelocityTimeSeries)-1)*deltaTime seconds
    # returns: float. angle of rotation in radians


    def theta(self, angle):
        return angle*self.timeDelta

    # angularVelocityTimeSeries: list<float>. Contains the discrete measurements for the angular velocity from 0 to (len(angularVelocityTimeSeries)-1)*deltaTime seconds
    # velocityTimeSeries: list<float>. Contains the discrete measurements for the linear velocity from 0 to (len(velocityTimeSeries)-1)*deltaTime seconds
    # returns: float. the x coordinate measured from the point of origin. In meters.
    def positionX(self, velocity, angle):
        return velocity*cos(angle)*self.timeDelta

    # angularVelocityTimeSeries: list<float>. Contains the discrete measurements for the angular velocity from 0 to (len(angularVelocityTimeSeries)-1)*deltaTime seconds
    # velocityTimeSeries: list<float>. Contains the discrete measurements for the linear velocity from 0 to (len(velocityTimeSeries)-1)*deltaTime seconds
    # returns: float. the x coordinate measured from the point of origin. In meters.
    def positionY(self, velocity, angle):
        return velocity*sin(angle)*self.timeDelta

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
    def init_mov(self):
        posX = self.current_x
        posY = self.current_y
        runTime = 0
        leftSpeed = (20*0.01)*900
        rightSpeed = (20*0.01)*900
        self.motorRight.run_timed(speed_sp=rightSpeed, time_sp=2*1000)
        self.motorLeft.run_timed(speed_sp=leftSpeed, time_sp=2*1000)
        while (runTime <= 2):
            wTimeSeries = []
            vTimeSeries = []
            angleTimeSeries = []
            self.previousTachoReadingLeft = self.currentTachoReadingLeft
            self.previousTachoReadingRight = self.currentTachoReadingRight
            sleep(self.timeDelta)
            runTime = runTime+self.timeDelta
            self.currentTachoReadingLeft = self.motorLeft.position
            self.currentTachoReadingRight = self.motorRight.position

            currentAngularVelocity = self.angularVelocity()
            currentVehicleVelocity = self.vehicleVelocity()

            self.angle = self.angle + self.theta(currentAngularVelocity)
            posX = posX + self.positionX(currentVehicleVelocity, self.angle)
            posY = posY + self.positionY(currentVehicleVelocity, self.angle)
        self.prev_x =self.current_x
        self.prev_y = self.current_y
        self.current_x = posX
        self.current_y = posY
        sleep(1)

    def inv_kin(self, tgt_x, tgt_y):
        self.init_mov()
        target_x = tgt_x
        target_y = tgt_y

        point1X = self.current_x
        point1Y = self.current_y
        point2X = self.prev_x
        point2Y = self.prev_y
        point3X = target_x
        point3Y = target_y

        print(self.current_x)
        print(self.current_y)
        print(self.prev_x)
        print(self.prev_y)
        print(target_x)
        print(target_y)
        sleep(20)
        # Find length of the first vector using points 1 and 2.
        v_a = (point2X - point1X)
        v_b = (point2Y - point1Y)
        vMag = sqrt(pow(v_a,2) + pow(v_b,2))

        # Find the length of the second vector using point 2 and 3.
        u_a = (point3X - point1X)
        u_b = (point3Y - point1Y)
        uMag = sqrt(pow(u_a,2) + pow(u_b,2))

        # Calculate the dot product of the numerator

        dotProd = (v_a * u_a) + (v_b * u_b)
        target_angle =  pi - acos(dotProd / (vMag * uMag))
        leftSpeed = (-10*0.01)*900
        rightSpeed = (10*0.01)*900
        self.motorRight.run_forever(speed_sp=rightSpeed)
        self.motorLeft.run_forever(speed_sp=leftSpeed)
        posX = self.current_x
        posY = self.current_y
        while target_angle > 0:
            print("angle: ")
            print(target_angle)
            self.previousTachoReadingLeft = self.currentTachoReadingLeft
            self.previousTachoReadingRight = self.currentTachoReadingRight
            sleep(self.timeDelta)
            self.currentTachoReadingLeft = self.motorLeft.position
            self.currentTachoReadingRight = self.motorRight.position
            currentAngularVelocity = self.angularVelocity()
            currentVehicleVelocity = self.vehicleVelocity()
            temp_ang = self.theta(currentAngularVelocity)
            if (temp_ang > 0):
                target_angle = target_angle - temp_ang
            else:
                target_angle = target_angle + temp_ang
            self.angle = self.angle + self.theta(currentAngularVelocity)
            #posX = posX + self.positionX(currentVehicleVelocity, self.angle)
            #posY = posY + self.positionY(currentVehicleVelocity, self.angle)
        self.motorRight.stop()
        self.motorLeft.stop()
        sleep(1)
        #self.prev_x =self.current_x
        #self.prev_y = self.current_y
        #self.current_x = posX
        #self.current_y = posY


        leftSpeed = (20*0.01)*900
        rightSpeed = (20*0.01)*900
        self.motorRight.run_forever(speed_sp=rightSpeed)
        self.motorLeft.run_forever(speed_sp=leftSpeed)
        while target_x > 0:
            print("X: ")
            print(target_x)
            self.angle = self.angle + self.theta(currentAngularVelocity)
            self.previousTachoReadingLeft = self.currentTachoReadingLeft
            self.previousTachoReadingRight = self.currentTachoReadingRight
            sleep(self.timeDelta)
            self.currentTachoReadingLeft = self.motorLeft.position
            self.currentTachoReadingRight = self.motorRight.position
            currentAngularVelocity = self.angularVelocity()
            currentVehicleVelocity = self.vehicleVelocity()
            self.angle = self.angle + self.theta(currentAngularVelocity)
            target_x = target_x - self.positionX(currentVehicleVelocity, self.angle)
            target_y = target_y - self.positionY(currentVehicleVelocity, self.angle)

        self.motorRight.stop()
        self.motorLeft.stop()
        self.prev_x =self.current_x
        self.prev_y = self.current_y
        sleep(1)


        print("current X: ", self.current_x)
        print("current Y: ", self.current_y)
