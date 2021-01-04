#!/usr/bin/python
# RUN ON LAPTOP USING PYTHON 3.6

#This class handles the Server side of the comunication between the laptop and the brick.

import socket
import time
from queue import Queue

class Server:
    def __init__(self,port):

        serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        host = "10.42.0.1"
        print ("host: ", host)
        serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        port = 9999
        serversocket.bind((host, port))


        serversocket.listen(5)
        print("here")
        self.cs,addr = serversocket.accept()
        print ("Connected to: " +str(addr) )


    def sendCoords(self, p1, p2, s1, s2, t1, t2, queue):

        print(str(p1) +  " " + str(p2) +  " "+ str(s1) +  " " + str(s2) + str(t1) +  " " + str(t2))
        data = str(p1)+"\t"+str(p2)+"\t"+str(s1)+"\t"+str(s2)+"\t"+str(t1)+"\t"+str(t2)
        print("Sending Data: (" + data + ") to robot.")
        self.cs.send(data.encode("UTF-8"))

        reply = self.cs.recv(128).decode("UTF-8")
        queue.put(reply)

    def sendAngles(self, base_angle, joint_angle, queue):
        #Format in which the client expects the data
        # angle1    angle2
        print(str(base_angle) +  " " + str(joint_angle))
        data = str(base_angle)+"\t"+str(joint_angle)
        print("Sending Data: (" + data + ") to robot.")
        self.cs.send(data.encode("UTF-8"))
        #Waiting for the client (ev3 brick) to let the server know
        #That it is done moving
        reply = self.cs.recv(128).decode("UTF-8")
        queue.put(reply)


    def sendTermination(self):
        self.cs.send("EXIT".encode("UTF-8"))
