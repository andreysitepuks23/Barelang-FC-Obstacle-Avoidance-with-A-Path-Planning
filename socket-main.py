# -*- coding: utf-8 -*-
"""
Created on Sat Apr  4 18:21:08 2020

@author: Andrey Karona Sitepu
"""

import time
import socket
import sys
import numpy as np
from os import system
import random

# Main configuration to receive data from kinematic
MAIN_UDP_IP = "127.0.0.1"
MAIN_UDP_IN_PORT = 5006
MAIN_UDP_OUT_PORT = 5005

totalLandmarks = 8#7#2
distanceRobotToLandmarks = np.zeros((totalLandmarks))
robotCoor = np.zeros((2))
ballCoor = np.zeros((2))
angle = 0
pathGrid = []


deltaTime = 0.5 #1 #update every "deltaTime" second

def main():
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((MAIN_UDP_IP, MAIN_UDP_IN_PORT))
    except socket.error:
        print ('Failed to create socket')
        sys.exit()
    
    totalDetectLandmarks 		    = 2
    arahGoal		                = 0
    distanceRobotToLandmarks[0] 	= 250
    distanceRobotToLandmarks[1] 	= 200
    distanceRobotToLandmarks[2] 	= 0
    distanceRobotToLandmarks[3] 	= 0
    distanceRobotToLandmarks[4] 	= 0
    distanceRobotToLandmarks[5] 	= 0
    distanceRobotToLandmarks[6] 	= 0
    distanceRobotToLandmarks[7] 	= 0
    ballDistance        			= 100
    imuRobot                        = 135
    panAngle            			= 45
    posRobotX 	 		            = 300
    posRobotY 			            = 50
    gridTarget                      = 21
    requestPath                     = 1
    gridRobot1                      = 34
    gridRobot2                      = 33
    gridRobot4                      = 34
    gridRobot5                      = 33

    
    # Timing value
    nowTime = 0
    lastTime = 0
    lastTime1 = 0
    loop = 0
    deltaTime1 = 5
    
    while True:
        nowTime = time.clock()
        timer = nowTime - lastTime
        timer1 = nowTime - lastTime1
        halfDeltaTime = deltaTime / 2.00
        halfDeltaTime1 = deltaTime1 / 2.00
        if timer > halfDeltaTime:
            system('cls')
            lastTime = nowTime
            loop += 1
            
            msgToMainProgram = "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}".format(int(totalDetectLandmarks),int(arahGoal),int(distanceRobotToLandmarks[0]),
                                                                                                    int(distanceRobotToLandmarks[1]),int(distanceRobotToLandmarks[2]),int(distanceRobotToLandmarks[3]),
                                                                                                    int(distanceRobotToLandmarks[4]),int(distanceRobotToLandmarks[5]),int(distanceRobotToLandmarks[6]),
                                                                                                    int(distanceRobotToLandmarks[7]),int(ballDistance),int(imuRobot),
                                                                                                    int(panAngle),int(posRobotX),int(posRobotY),
                                                                                                    int(gridTarget),int(requestPath),int(gridRobot1),
                                                                                                    int(gridRobot2),int(gridRobot4),int(gridRobot5))
            sock.sendto(msgToMainProgram.encode('utf-8', 'strict'),(MAIN_UDP_IP, MAIN_UDP_OUT_PORT))
            print("Data Send To Particle MCL = ", msgToMainProgram)
            data, _ = sock.recvfrom(1024)
            dataDecode = data.decode('utf-8', 'strict')
            strDataFromKinematic = dataDecode.split(",")
            #print("Data From Particle Filter = ", strDataFromKinematic)
            robotCoor[0]            = strDataFromKinematic[0]
            robotCoor[1]            = strDataFromKinematic[1]
            angle                   = strDataFromKinematic[2]
            ballCoor[0]             = strDataFromKinematic[3]
            ballCoor[1]             = strDataFromKinematic[4]
            pathGrid                = strDataFromKinematic[5:len(strDataFromKinematic)]
            
            print("robotCoorX\t\t\t = ", robotCoor[0])
            print("robotCoorY\t\t\t = ", robotCoor[1])
            print("angle\t\t\t\t = ", angle)
            print("ballCoorX\t\t\t = ", ballCoor[0])
            print("ballCoorY\t\t\t = ", ballCoor[1])
            print("Path Grid\t\t\t = ", pathGrid)
            
            
            
            if timer1 > halfDeltaTime1:
                lastTime1 = nowTime    
                #drawObstacle(obsCounter)
                requestPath = not(requestPath)
                #imuRobot = random.randint(-180,179)
                #drawPointAndLine()
            
if __name__ == "__main__":
        print ('Running BarelangFC - MCL Send Data')
        main()