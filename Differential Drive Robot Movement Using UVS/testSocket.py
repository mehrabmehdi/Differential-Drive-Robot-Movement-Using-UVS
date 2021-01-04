from client import Client
from ev3dev2 import *
from math import *
import socket
from threading import Event, Thread
from time import sleep
from ev3dev2.motor import LargeMotor, OUTPUT_C, OUTPUT_D, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.led import Leds
from DDrive import DDrive






#This routine will run on the lego Brick. It initalizes the client and will listen
#for commands from the server.

#Instantiate motors and sensors
base_motor = LargeMotor(OUTPUT_C)
joint_motor = LargeMotor(OUTPUT_D)
#Attempt to connect to server
client = Client(9999)
#Reset motors to make sure the encoder counter starts at 0 for both.
#joint_motor.reset()
#base_motor.reset()


#drive =DDrive(base_motor, joint_motor)
#A = Arm(base_motor, joint_motor)
#By default safety mode is off on the brick
while True:
    #Block until a command/message from the server is received
    data = str(client.pollData())
    if(data == 'EXIT'):
        #Terminate the routine on the client
        base_motor.stop()
        joint_motor.stop()
        break
    else:

        #robot_x, robot_y, target_x, target_y = map(float,map(str,data.split('\t')))


        #After the proper rotation angle has been selected, we send the move command to the motorsbase
        #drive.current_x = robot_x
        #drive.current_y = robot_y
        # print("here")
        # print(robot_x)
        # print(robot_y)
        # print(target_x)
        # print(target_y)
        #drive.inv_kin(target_x, target_y)
        left , right = map(float,map(str,data.split('\t')))
        leftSpeed = (left*0.01)*900
        rightSpeed = (right*0.01)*900
        print(left)
        print(right)
        base_motor.run_timed(speed_sp=rightSpeed, time_sp=0.25*1000)
        joint_motor.run_timed(speed_sp=leftSpeed, time_sp=0.25*1000)

        print( "RUN IS SUCCESSFULL")

        client.sendDone()

base_motor.stop()
joint_motor.stop()
