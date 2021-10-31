#Step To Play This Simulator
#conda activate python_robotics
#cd Desktop\Riset\PyPathPlaning\PythonRobotics-master\PathPlanning\BFC

#from flask import Flask, render_template, Response, request
from scipy.spatial import distance
from numpy.random import uniform, normal
import time
import socket
import sys
import numpy as np
import scipy.stats
import math
import cv2

#tambahan Path Planning
from os import system
import os
import random

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

#import rrt and A-Star library
try:
    from rrt import RRT
    from a_star import AStarPlanner
except ImportError:
    raise

#Set the Method and Animation Display
show_animation = False #Please Always set False for Speed Processing
pathMode = 1; # 0 = RRT_Smoothing, 1 = A-Star,
    
#tambahan Path Planning
startRobot = [750,350] #cm
goalRobot = [0,50] #cm
area_rand = startRobot + goalRobot
"""
obsCoor1 = [350,450] #cm
obsCoor2 = [350,150] #cm
obsCoor3 = [550,350] #cm
obsCounter = 0
"""
obstacle = [100, 200, 10]
obstacleGrid = [100, 200, 10]
#gObstacle = [100, 200, 10]
tempArray = np.zeros(())
tempPointOut = np.zeros(())
gridSendToMain = []
#gridPoint = []
gridObstacleList = []
tempArrayGrid = np.zeros(())
remove_list = ['0','0.0']
remove_list1 = [0,0.0]
removeSameList = []
gridAfter = []
tempData = []

# Path Planning Parameter
obstacleList = []
pointOutX = []
pointOutY = []
gridTarget = 0
posRobotX = 0
posRobotY = 0

#Variable Switch Update Path
requestPath = 0
lastRequestPath = 0
requestState = 0
imuCurrentHeading = 0

# Definisi ID robot
robotID = 3

# Set False for real localization
simulationMode = False
headingFromIMU = True

# Main configuration to receive data from kinematic
MAIN_UDP_IP = "127.0.0.1"
MAIN_UDP_IN_PORT = 5005
MAIN_UDP_OUT_PORT = 5006

# Configuration in Cm
fieldLength = 900
fieldWidth = 600

# Particles and landmarks
#totalParticles = 100
totalParticles = 100
totalLandmarks = 8#7#2
deltaTime = 0.5 #1 #update every "deltaTime" second

mapImage = np.zeros((800,1100,3), np.uint8)

# Global Variable
arahGoal = np.zeros((0))
robotGlobalPosition = np.zeros((3))
robotInitialPosition = np.zeros((3))
robotLocalPosition = np.zeros((3))
odometryPosition = np.zeros((2))
robotCoorToMain = np.zeros((2))
ballCoorToMain = np.zeros((2))

# Landmarks position 2D array
landmarksPosition = np.zeros((totalLandmarks, 2))
drawLandmarksPosition = np.zeros((totalLandmarks,2))

# Particles position
particlesGlobalPosition = np.zeros((totalParticles, 3))
particlesLocalPosition = np.zeros((totalParticles, 3))
particlesInitialPosition = np.zeros((totalParticles, 3))

# Estimate position
estimatePosition = np.zeros((3))
estimateInitialPosition = np.zeros((3))
estimateLocalPosition = np.zeros((3))
ballEstimatePosition = np.zeros((2))

distanceRobotToLandmarks = np.zeros((totalLandmarks))
distanceParticlesToLandmarks = np.zeros((totalParticles, totalLandmarks))
particlesWeight= np.zeros((totalParticles))

velFromKinematic = np.zeros((3))

realVelocity = np.zeros([3])


									
#app = Flask(__name__)
# http://mattrichardson.com/Raspberry-Pi-Flask/
#@app.route('/')
#def index():
#    """Video streaming home page."""
#    templateData = { 
#            'robotid' : str(robotID),
#        }
#    return render_template('index.html', **templateData)
									
#@app.route('/video_feed')
#def video_feed():
#    """Video streaming route. Put this in the src attribute of an img tag."""
#    return Response(main(), mimetype='multipart/x-mixed-replace; boundary=frame')

def threadLocalization():
        os.system("./mjpgLocalization.sh")

# Convert robot velocity to cm/s
# Loaded from calibrated value
def convertVel(robotId, inputVel):
    outputVel = np.zeros((3))
    if robotId == 1:
        if inputVel[0] == 0:
            outputVel[0] = 0
        else:
            outputVel[0] = 355.27 * inputVel[0] - 0.4811

        if inputVel[1] == 0:
            outputVel[1] = 0
        else:
            outputVel[1] = 389.51 * inputVel[1] + 0.1839

        if inputVel[2] == 0:
            outputVel[2] = 0
        else:
            outputVel[2] = 124.78 * inputVel[2] + 1.366
    elif robotId == 3:				#JIKA ROBOT 1
        if inputVel[0] == 0:			#JIKA INPUTVEL[0] == 0
            outputVel[0] = 0			#OUTPUT[0] == 0
        else:							#SELAIN
            outputVel[0] = 355.27 * inputVel[0] - 0.4811	#OUTPUT[0] = RUMUS REGRESI X * INPUT

        if inputVel[1] == 0:   			#JIKA INPUTVEL[1] == 0
            outputVel[1] = 0			#OUTPUT[1] == 0
        else:							#SELAIN
            outputVel[1] = 389.51 * inputVel[1] + 0.1839	#OUTPUT[1] = RUMUS REGRESI Y * INPUT

        if inputVel[2] == 0:			#JIKA INPUTVEL[2] == 0
            outputVel[2] = 0			#OUTPUT[2] == 0
        else:							#SELAIN
            outputVel[2] = 124.78 * inputVel[2] + 1.366		#OUTPUT[2] = RUMUS REGRESI THETHA

    elif robotId == 5:
        if inputVel[0] == 0:
            outputVel[0] = 0
        else:
            outputVel[0] = 315.95 * inputVel[0] - 0.5579

        if inputVel[1] == 0:
            outputVel[1] = 0
        else:
            outputVel[1] = 338.71 * inputVel[1] + 0.9102

        if inputVel[2] == 0:
            outputVel[2] = 0
        else:
            outputVel[2] = 131.66 * inputVel[2] + 0.9137
    return outputVel

def worldCoorToImageCoor(x, y):		#FUNGSI DRAW KOORDINAT SUDUT - SUDUT LAPANGAN JIKA TIDAK PAKE FILE MAPIMAGE.JPG
    x = x + 100				#OUTPUT X = X + 100 AKAN BERGESER KE KANAN + 100 PIXEL
    y = 800- (y + 100)			#OUTPUT Y = 800 - (Y + 100) FLIP MODE DRAW
    return x, y				#RETURN HASIL

def sendToMain():
    robotCoorToMain[0] = estimatePosition[0] - 450
    robotCoorToMain[1] = 300 - estimatePosition[1]
    ballCoorToMain[0] = ballEstimatePosition[0] - 450
    ballCoorToMain[1] = 300 - ballEstimatePosition[1]

########################################################    
#########Tambahan Path Planning RRT_Smoothing###########
########################################################
def get_path_length(path):
    le = 0
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        d = math.sqrt(dx * dx + dy * dy)
        le += d

    return le

def get_target_point(path, targetL):
    le = 0
    ti = 0
    lastPairLen = 0
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        d = math.sqrt(dx * dx + dy * dy)
        le += d
        if le >= targetL:
            ti = i - 1
            lastPairLen = d
            break

    partRatio = (le - targetL) / lastPairLen

    x = path[ti][0] + (path[ti + 1][0] - path[ti][0]) * partRatio
    y = path[ti][1] + (path[ti + 1][1] - path[ti][1]) * partRatio

    return [x, y, ti]

def line_collision_check(first, second, obstacleList):
    # Line Equation

    x1 = first[0]
    y1 = first[1]
    x2 = second[0]
    y2 = second[1]

    try:
        a = y2 - y1
        b = -(x2 - x1)
        c = y2 * (x2 - x1) - x2 * (y2 - y1)
    except ZeroDivisionError:
        return False

    for (ox, oy, size) in obstacleList:
        d = abs(a * ox + b * oy + c) / (math.sqrt(a * a + b * b))
        if d <= size:
            return False

    return True  # OK

def path_smoothing(path, max_iter, obstacle_list):
    le = get_path_length(path)

    for i in range(max_iter):
        # Sample two points
        pickPoints = [random.randint(1, int(le)), random.randint(1, int(le))]
        pickPoints.sort()
        first = get_target_point(path, pickPoints[0])
        second = get_target_point(path, pickPoints[1])

        if first[2] <= 0 or second[2] <= 0:
            continue

        if (second[2] + 1) > len(path):
            continue

        if second[2] == first[2]:
            continue

        # collision check
        if not line_collision_check(first, second, obstacle_list):
            continue

        # Create New path
        newPath = []
        newPath.extend(path[:first[2] + 1])
        newPath.append([first[0], first[1]])
        newPath.append([second[0], second[1]])
        newPath.extend(path[second[2] + 1:])
        path = newPath
#        print(path)
        le = get_path_length(path)

    return path
########################################################
########################################################

def convertGridToCoor(grid): #The value of the grid should int
    if grid > 0 and grid <= 6:
        baseGrid = 6                        #---------------GRID FIELD--------------#
    elif grid > 6 and grid <= 12:           # - # - # - # - # - # - # - # - # - # - #
        baseGrid = 12                      #|                                       |#
    elif grid > 12 and grid <= 18:          #   1   7   13  19  25  31  37  43  49  #
        baseGrid = 18                      #|                                       |#
    elif grid > 18 and grid <= 24:          #   2   8   14  20  26  32  38  44  50  #
        baseGrid = 24                      #|                                       |#
    elif grid > 24 and grid <= 30:          #   3   9   15  21  27  33  39  45  51  #
        baseGrid = 30                      #|                                       |#
    elif grid > 30 and grid <= 36:          #   4   10  16  22  28  34  40  44  52  #
        baseGrid = 36                      #|                                       |#
    elif grid > 36 and grid <= 42:          #   5   11  17  23  29  35  41  44  53  #
        baseGrid = 42                      #|                                       |#
    elif grid > 42 and grid <= 48:          #   6   12  18  24  30  36  42  45  54  #  
        baseGrid = 48                      #|                                       |#
    elif grid > 48 and grid <= 54:          # - # - # - # - # - # - # - # - # - # - #   
        baseGrid = 54
      
    if grid %6 == 0:
        tempCoorX = (math.floor(grid/6) * 100) - 50
        tempCoorY = ((baseGrid - grid) * 100) + 50
        
    else:
        tempCoorX = (math.floor(grid/6) * 100) + 50
        tempCoorY = ((baseGrid - grid) * 100) + 50

    return tempCoorX, tempCoorY #The result is the center coordinates of the grid

def convertCoorToGrid(x, y): #X and Y Coordinates should in cm
    tempGridX =  math.ceil(x/100) * 6
    tempGridY = math.floor(y/100)
    
    grid = tempGridX - tempGridY
    if grid < 1:
        grid = 1
    
    return grid #result is the grid field

def drawObstacle(count):
    global obstacleList
    global gridObstacleList
    #print(count)
    #print(gridObstacleList)
    tempArray = np.zeros((count,3))
    if count > 0 and len(gridObstacleList) > 0:
        #print(" ")
        #print(" ")
        #print("\t\tOBSTACLE: %d\t    X\t    Y\t    S"%count)
        if simulationMode == False:
            for i in range(count):
                #print(i)
                #print(len(gridObstacleList))
                """
                obstacle[0] = random.randint(500, 500)
                obstacle[1] = random.randint(300, 300)
                obstacle[2] = random.randint(50, 50)
                gridObs = convertCoorToGrid(obstacle[0], obstacle[1])
                """
                xPointObs, yPointObs = convertGridToCoor(int(gridObstacleList[i]))
                obstacleGrid[0] = xPointObs
                obstacleGrid[1] = yPointObs
                obstacleGrid[2] = 50
                #xObs,yObs = worldCoorToImageCoor(obstacle[0],obstacle[1])
                #print("obstaCoordinate (X Y S) %d\t= (%.2f %.2f %.2f)" %(i, obstacle[0], obstacle[1], obstacle[2]))
                xObs,yObs = worldCoorToImageCoor(xPointObs,yPointObs)
                #print("obstaCoordinate (X Y S) %d\t= (%.2f %.2f %.2f)" %(i, xPointObs, yPointObs, obstacle[2]))
                tempArray[i] = obstacleGrid
                cv2.circle(mapImage,(xObs, yObs), 50, (100,100,100), -1)
                obstacleList = tempArray.tolist()
                #print(obstacleList)
        else:
            #"totalDetectLandmark,arahGoal,distanceRobotToLandmarks0,distanceRobotToLandmarks1,distanceRobotToLandmarks2,distanceRobotToLandmarks3,distanceRobotToLandmarks4,distanceRobotToLandmarks5,distanceRobotToLandmarks6,distanceRobotToLandmarks7,ballDistance,panAngle,robotX,robotY,gridGoal,requestPath,gridObs1,gridObs2,gridObs3,gridObs4"
            print("")
        
def searchPath():
    global gridSendToMain
    global obstacleList
    global pointOutX
    global pointOutY
    #obstacleList = [(obsCoor1[0], obsCoor1[1], 50), (obsCoor2[0], obsCoor2[1], 50), (obsCoor3[0], obsCoor3[1], 50)]
    #gridStart = convertCoorToGrid(startRobot[0], startRobot[1])
    
    #RRT_Smoothing
    if pathMode == 0:
        gridStart = convertCoorToGrid(posRobotX + robotInitialPosition[0], posRobotY + robotInitialPosition[1])
        x, y = convertGridToCoor(gridStart)
        x1, y1 = convertGridToCoor(int(gridTarget))
        rrt = RRT(start = [x, y], goal = [x1, y1],
            rand_area=[min(area_rand), max(area_rand)], obstacle_list=obstacleList)
        path = rrt.planning(animation=show_animation)
        #print(path)
    
        if path is None:
            print("CANNOT FIND PATH")
        else:
            # Path smoothing
            maxIter = 500
            #smoothedPath = path # RRT with out smoothing
            smoothedPath = path_smoothing(path, maxIter, obstacleList) # RRT with smoothing
            smoothedPath.reverse()
            xPointCoor, yPointCoor = zip(*smoothedPath)
            pointCoordinateX = list(dict.fromkeys(np.around(xPointCoor[1:len(xPointCoor)-1])))
            pointCoordinateY = list(dict.fromkeys(np.around(yPointCoor[1:len(yPointCoor)-1])))
            pointCoordinateX1 = np.around(xPointCoor[0:len(xPointCoor)-1])
            pointCoordinateY1 = np.around(yPointCoor[0:len(yPointCoor)-1])
            pointOutX = pointCoordinateX
            pointOutY = pointCoordinateY
            tempArrayGrid = np.zeros((len(pointCoordinateX1),1))
            for i in range(len(pointCoordinateX1)):
                tempArrayGrid[i] = convertCoorToGrid(int(pointCoordinateX1[i]), int(pointCoordinateY1[i]))
            #print("Tem array = ", tempArrayGrid)
            """
            #draw line pathPlaning
            for i in range(len(tempArrayGrid)-1):
                #print(i)
                if i%2 == 0:
                    xGridDrawS, YGridDrawS = convertGridToCoor(int(tempArrayGrid[i]))
                    xGridDrawE, YGridDrawE = convertGridToCoor(int(tempArrayGrid[i+1]))
                    xS, yS = worldCoorToImageCoor(xGridDrawS, YGridDrawS)
                    xE, yE = worldCoorToImageCoor(xGridDrawE, YGridDrawE)
                    cv2.line(mapImage, (xS,yS), (xE,yE), (100,100,0), 3)
                    
            #a, b, c= zip(*obstacleList)
            #print("xObstacle  = ", a)
            #print("yObstacle  = ", b)
            #print("sObstacle  = ", c)    
            #print("\t\tPOINT\t: %d\t    X\t    Y"%len(pointCoordinateX))
            
            #Robot Start and Goal Coor
            xStart,yStart = worldCoorToImageCoor(x,y)
            cv2.circle(mapImage,(xStart, yStart), 15, (255 ,127,127), -1)
            xGoal,yGoal = worldCoorToImageCoor(x1,y1)
            cv2.circle(mapImage,(xGoal, yGoal), 10, (255,255,0), -1)
            """
            
    #A-Star
    elif pathMode == 1:
        sx = (posRobotX + robotInitialPosition[0])/100
        sy = (posRobotY + robotInitialPosition[1])/100
        xG, yG = convertGridToCoor(int(gridTarget))
        gx = xG/100
        gy = yG/100
        grid_size = 0.5  # [m]
        robot_radius = 0.5  # [m]
        
        # set obstable positions
        ox, oy = [], []
        ox.append(5)
        oy.append(7)
        
        for i in range(0, 10):
            ox.append(i)
            oy.append(0.0)
        
        for i in range(0, 7):
            ox.append(9.0)
            oy.append(i)
        
        for i in range(0, 10):
            ox.append(i)
            oy.append(6.0)
            
        for i in range(0, 7):
            ox.append(0.0)
            oy.append(i)
        
        for i in range(len(gridObstacleList)):
            x, y = convertGridToCoor(int(gridObstacleList[i]))
            ox.append(x/100)
            oy.append(y/100)
        
#        try:            
        a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
        
        #print("ox = ",ox)
        #print("oy = ",oy)
        #print("grid_size = ",grid_size)
        #print("robot_radius = ",robot_radius)
        """
        print("sx = ",sx)
        print("sy = ",sy)
        print("gx = ",gx)
        print("gy = ",gy)
        """
        rx, ry = a_star.planning(sx, sy, gx, gy)
        tempArrayGrid = np.zeros(len(rx))
        """
        print("rx = ",rx)
        print("ry = ",ry)
        """
        for i in range(len(rx)):
            a = convertCoorToGrid(int(rx[i]*100), int(ry[i]*100))
            if a not in tempArrayGrid:
                tempArrayGrid[i] = int(a)
                #print(tempArrayGrid[i])
        #temData = np.trim_zeros(tempArrayGrid)
        #print(temData)
        #print(tempArrayGrid.tolist())
        gridSendToMain = tempArrayGrid.tolist()
        #gridSendToMain = tempArrayGrid
        #gridSendToMain = [round(x) for x in gridSendToMain]
        #gridSendToMain = list(dict.fromkeys(tempArrayGrid.tolist()))
        gridSendToMain.reverse()
        #print(gridSendToMain)
        gridSendToMain = [i for i in gridSendToMain if i not in remove_list1]
        gridSendToMain = [i for i in gridSendToMain if i not in gridAfter]
        #print(gridSendToMain)
        #print(type(gridSendToMain))
#        except:
#            pass
        
def calculateCoorWithAngle(x, y, a, l):
    thetaAngle = np.radians(a)
    cosAngle, sinAngle = np.cos(thetaAngle), np.sin(thetaAngle)
    resultX = int(x + (l * cosAngle))
    resultY = int(y + (l * sinAngle))
    
    return resultX, resultY
        
def drawRobotAndTarget():
    #global imuCurrentHeading
    gridNow = convertCoorToGrid(posRobotX + robotInitialPosition[0], posRobotY + robotInitialPosition[1])
    x, y = convertGridToCoor(gridNow)
    #x1, y1 = convertGridToCoor(int(gridTarget))
    xAngleRobot, yAngleRobot = calculateCoorWithAngle(x, y, robotLocalPosition[2], 50)
    xGlobalRobot, yGlobalRobot = worldCoorToImageCoor(x, y)
    xGlobalRobotHeading, yGlobalRobotHeading = worldCoorToImageCoor(xAngleRobot, yAngleRobot)
    cv2.arrowedLine(mapImage, (xGlobalRobot, yGlobalRobot), (xGlobalRobotHeading, yGlobalRobotHeading), (0, 0, 255), 13, 8, 0, 0.3)

def main():
    global pointCoordinateX
    global area_rand
    global gridSendToMain
    global requestPath
    global lastRequestPath
    global requestState
    global gridObstacleList
    global totalDetectLandmarks
    global arahGoal
    global posRobotX
    global posRobotY
    global gridTarget
    global requestPath
    global removeSameList
    
    # Set initial location of robot, Just for simulation
    robotInitialPosition[0] = 450 # X
    robotInitialPosition[1] = 300 # Y
    robotInitialPosition[2] = 0   # Heading
    robotGlobalPosition[:] = 0		# DEKLARASI ARRAY DENGAN NILAI 0 TANPA BATAS UNTUK GLOBAL POSITION
    robotLocalPosition[:] = 0		# DEKLARASI ARRAY DENGAN NILAI 0 TANPA BATAS UNTUK LOCAL POSITION
    odometryPosition[0] = 0
    odometryPosition[1] = 0


    # Initialize landmark position (left and right goal pole)
    landmarksPosition[0,0] = 900	#GAWANG         RG      
    landmarksPosition[0,1] = 430	#GAWANG         RG
    landmarksPosition[1,0] = 900	#GAWANG         LG
    landmarksPosition[1,1] = 170	#GAWANG         LG
    landmarksPosition[2,0] = 700        #LCROSS         RIGHT
    landmarksPosition[2,1] = 550        #LCROSS         RIGHT
    landmarksPosition[3,0] = 700        #LCROSS         LEFT
    landmarksPosition[3,1] = 50         #LCROSS         LEFT
    #landmarksPosition[4,0] = 750        #PENALTY
    #landmarksPosition[4,1] = 300        #PENALTY
    #landmarksPosition[5,0] = 450        #XCROSS         RIGHT
    #landmarksPosition[5,1] = 375        #XCROSS         RIGHT
    landmarksPosition[4,0] = 450        #XCROSS         RIGHT
    landmarksPosition[4,1] = 375        #XCROSS         RIGHT
    landmarksPosition[5,0] = 450        #XCROSS         LEFT
    landmarksPosition[5,1] = 225        #XCROSS         LEFT
    #landmarksPosition[7,0] = 450        #TCROSS         RIGHT
    #landmarksPosition[7,1] = 600        #TCROSS         RIGHT
    landmarksPosition[6,0] = 450        #TCROSS         RIGHT
    landmarksPosition[6,1] = 600        #TCROSS         RIGHT
    #landmarksPosition[8,0] = 450        #TCROSS         LEFT
    #landmarksPosition[8,1] = 0          #TCROSS         LEFT
    landmarksPosition[7,0] = 450        #TCROSS         LEFT
    landmarksPosition[7,1] = 0          #TCROSS         LEFT


    velFromKinematic[0] = 0.00		#INIT VALUE WALK X
    velFromKinematic[1] = 0.00		#INIT VALUE WALK Y
    velFromKinematic[2] = 0.00		#INIT VALUE WALK A

    #imuInitHeading = 0			#INIT VALUE HEADING
    imuCurrentHeading = 0		#INIT VALUE HEADING SEBELUMNYA

    arahGoal = 0			#Arah gawang lawan (0) gawang tim (1)
    ballDistance = 0			#INIT VALUE JARAK BOLA
    panAngle = 0			#INIT VALUE SUDUT SERVO PAN

    posRobotX = 0
    posRobotY = 0
    totalDetectLandmarks = 0

    #phi = 3.1428571428571428571428571428571
    defineInitialPosition = True
    if defineInitialPosition == True:
        # Create 90 percent random particles from defined initial position and 10 percent from random uniform
        estimateInitialPosition[0] = 450 # X
        estimateInitialPosition[1] = 300 # Y
        estimateInitialPosition[2] = 0 # Heading
        #estimateInitialPosition[2] = 120 # Heading utara
        #estimateInitialPosition[2] = 300 # Heading selatan

        _10PercentParticle = int(totalParticles * 0.1)

        for i in range (0, _10PercentParticle):
            particlesInitialPosition[i,0] = uniform(0, fieldLength)
            particlesInitialPosition[i,1] = uniform(0, fieldWidth)
            particlesInitialPosition[i,2] = uniform(0, 360)
            particlesGlobalPosition[i,:] = 0
            particlesLocalPosition[i,:] = 0

        _90PercentParticle = totalParticles - _10PercentParticle

        for i in range (_10PercentParticle+1, totalParticles):
            particlesInitialPosition[i,0] = normal(estimateInitialPosition[0], 30)
            particlesInitialPosition[i,1] = normal(estimateInitialPosition[1], 30)
            particlesInitialPosition[i,2] = normal(estimateInitialPosition[2], 10)
            particlesGlobalPosition[i,:] = 0
            particlesLocalPosition[i,:] = 0
    else:
        # Create random uniform position of particles
        particlesInitialPosition[:,0] = uniform(0, fieldLength, size=totalParticles)
        particlesInitialPosition[:,1] = uniform(0, fieldWidth, size=totalParticles)
        particlesInitialPosition[:,2] = uniform(0, 360, size=totalParticles)

    # Zero all global and local position of particles
    particlesGlobalPosition[:,:] = 0
    particlesLocalPosition[:,:] = 0

    # Create UDP client to receive data from kinematic
    if simulationMode == False:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind((MAIN_UDP_IP, MAIN_UDP_IN_PORT))
        except socket.error:
            print ('Failed to create socket')
            sys.exit()

    # Timing value
    nowTime = 0
    lastTime = 0
    loop = 0

    #pthreadLocalization = threading.Thread(target=threadLocalization)
    #pthreadLocalization.start()

    while True:
        nowTime = time.clock()
        timer = nowTime - lastTime
        halfDeltaTime = deltaTime / 2.00
        # Update every 0.5 * deltatime
        if timer > halfDeltaTime:
            system('cls')
            lastTime = nowTime
            loop += 1
            #print ('Runtime : {} s'.format(deltaTime*loop))

            mapFromFile = False
            if mapFromFile == True:
                # image tidak clear
                mapImage[:] = cv2.imread('mapImage.jpg') 
            else:
                mapImage[:] = (0, 255, 0)
                cv2.rectangle(mapImage, (100,100), (1000,700), (255,255,255), 3) # Garis Luar
                cv2.rectangle(mapImage, (40,530), (100,270), (255,255,255), 3) # Garis Luar Gawang Kiri
                cv2.rectangle(mapImage, (1000,530), (1060,270), (255,255,255), 3) # Garis Luar Gawang Kanan
                cv2.rectangle(mapImage, (100,650), (300,150), (255,255,255), 3) # Garis Kotak Pinalti Kiri
                cv2.rectangle(mapImage, (100,550), (200,250), (255,255,255), 3) # Garis Goal Area Kiri
                cv2.rectangle(mapImage, (800,650), (1000,150), (255,255,255), 3) # Garis Kotak Pinalti Kanan
                cv2.rectangle(mapImage, (900,550), (1000,250), (255,255,255), 3) # Garis Goal Area Kanan
                cv2.line(mapImage, (550,100), (550,700), (255,255,255), 3) # Garis Tengah
                cv2.circle(mapImage, (550,400), 75, (255,255,255), 3) # Lingkaran Tengah
                cv2.circle(mapImage, (250,400), 3, (255,255,255), 5) # Titik Pinalti Kiri
                cv2.circle(mapImage, (850,400), 3, (255,255,255), 5) # Titik Pinalti Kanan

            showGrid = True
            if showGrid == True:
                cv2.line(mapImage, (100,200), (1000,200), (0,0,0), 1)
                cv2.line(mapImage, (100,300), (1000,300), (0,0,0), 1)
                cv2.line(mapImage, (100,400), (1000,400), (0,0,0), 1)
                cv2.line(mapImage, (100,500), (1000,500), (0,0,0), 1)
                cv2.line(mapImage, (100,600), (1000,600), (0,0,0), 1)
                
                cv2.line(mapImage, (200,100), (200,700), (0,0,0), 1)
                cv2.line(mapImage, (300,100), (300,700), (0,0,0), 1)
                cv2.line(mapImage, (400,100), (400,700), (0,0,0), 1)
                cv2.line(mapImage, (500,100), (500,700), (0,0,0), 1)
                cv2.line(mapImage, (600,100), (600,700), (0,0,0), 1)
                cv2.line(mapImage, (700,100), (700,700), (0,0,0), 1)
                cv2.line(mapImage, (800,100), (800,700), (0,0,0), 1)
                cv2.line(mapImage, (900,100), (900,700), (0,0,0), 1)
                for i in range(1, 55):
                    xText, yText = convertGridToCoor(i)
                    x, y = worldCoorToImageCoor(xText-15, yText-10)
                    textGrid = "%d"%i
                    cv2.putText(mapImage, textGrid,(x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 1, cv2.LINE_AA)

                textLine = "(0,0)"
                x, y = worldCoorToImageCoor(-50,0)
                cv2.putText(mapImage, textLine, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.LINE_AA) #OpenCV 3.x
                #cv2.putText(mapImage, textLine, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.CV_AA) #OpenCV 2.x

                textLine = "(0,600)"
                x, y = worldCoorToImageCoor(-50,600)
                cv2.putText(mapImage, textLine, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.LINE_AA) #OpenCV 3.x
                #cv2.putText(mapImage, textLine, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.CV_AA) #OpenCV 2.x

                textLine = "(900,600)"
                x, y = worldCoorToImageCoor(900,600)
                cv2.putText(mapImage, textLine, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.LINE_AA) #OpenCV 3.x
                #cv2.putText(mapImage, textLine, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.CV_AA) #OpenCV 2.x

                textLine = "(900,0)"
                x, y = worldCoorToImageCoor(900,0)
                cv2.putText(mapImage, textLine, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.LINE_AA) #OpenCV 3.x
                #cv2.putText(mapImage, textLine, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.CV_AA) #OpenCV 2.x
                
                if pathMode == 0:
                    textLine = "Path Mode : RRT Smoothing"
                    x, y = worldCoorToImageCoor(700,680)
                    cv2.putText(mapImage, textLine, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.LINE_AA) #OpenCV 3.x
                    
                elif pathMode == 1:
                    textLine = "Path Mode : A-Star"
                    x, y = worldCoorToImageCoor(700,680)
                    cv2.putText(mapImage, textLine, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.LINE_AA) #OpenCV 3.x

                drawLandmark = True
                if drawLandmark == True:
                    for i in range(totalLandmarks):
                        x, y = worldCoorToImageCoor(int(landmarksPosition[i,0]), int(landmarksPosition[i,1]))
                        cv2.circle(mapImage,(x, y), 10, (127,0,127), -1)

            # Get data from kinematic
            if simulationMode == False:
                data, _ = sock.recvfrom(1024) # buffer size is 1024 bytes
                dataDecode = data.decode('utf-8', 'strict')
                strDataFromKinematic = dataDecode.split(",")
                totalDetectLandmarks 		    = int(strDataFromKinematic[0])
                arahGoal		                = int(strDataFromKinematic[1])     
                distanceRobotToLandmarks[0] 	= float(strDataFromKinematic[2])	# 
                distanceRobotToLandmarks[1] 	= float(strDataFromKinematic[3])	# 
                distanceRobotToLandmarks[2] 	= float(strDataFromKinematic[4])	# 
                distanceRobotToLandmarks[3] 	= float(strDataFromKinematic[5])	# 
                distanceRobotToLandmarks[4] 	= float(strDataFromKinematic[6])	# 
                distanceRobotToLandmarks[5] 	= float(strDataFromKinematic[7])	# 
                distanceRobotToLandmarks[6] 	= float(strDataFromKinematic[8])	# 
                distanceRobotToLandmarks[7] 	= float(strDataFromKinematic[9])	# 
                ballDistance        			= float(strDataFromKinematic[10])	#
                imuCurrentHeading               = float(strDataFromKinematic[11])   #
                panAngle            			= float(strDataFromKinematic[12])	# Pan Angle (Robot)
                posRobotX 	 		            = float(strDataFromKinematic[13])	#Position RObot X From Odometry+initialPos main
                posRobotY 			            = float(strDataFromKinematic[14])*-1#Position Robot Y From Odometry+initialPos main
                gridTarget                      = int(strDataFromKinematic[15])
                requestPath                     = int(strDataFromKinematic[16])
                gridObstacleList                = list(dict.fromkeys(strDataFromKinematic[17:21]))
                removeSameList                  = gridObstacleList
                gridObstacleList                = [i for i in gridObstacleList if i not in remove_list]
                
                tempGridRemove = np.zeros(len(removeSameList))
                for i in range(len(removeSameList)):
                    tempGridRemove[i] = removeSameList[i]
                    gridAfter = list(dict.fromkeys(tempGridRemove.tolist()))    
                
                if pathMode == 0:
                    print("\t\t\tRRT Path Planning")
                elif pathMode == 1:
                    print("\t\t\tA-Star Path Planning")
                print("totalDetectLandmarks\t\t = ", totalDetectLandmarks)
                print("arahGoal\t\t\t = ", arahGoal)
                print("distanceRobotToLandmarks 1\t = ", distanceRobotToLandmarks[0])
                print("distanceRobotToLandmarks 2\t = ", distanceRobotToLandmarks[1])
                print("distanceRobotToLandmarks 3\t = ", distanceRobotToLandmarks[2])
                print("distanceRobotToLandmarks 4\t = ", distanceRobotToLandmarks[3])
                print("distanceRobotToLandmarks 5\t = ", distanceRobotToLandmarks[4])
                print("distanceRobotToLandmarks 6\t = ", distanceRobotToLandmarks[5])
                print("distanceRobotToLandmarks 7\t = ", distanceRobotToLandmarks[6])
                print("distanceRobotToLandmarks 8\t = ", distanceRobotToLandmarks[7])
                print("balDistance\t\t\t = ", ballDistance)
                print("imuCurrentHeading\t\t = ", imuCurrentHeading)
                print("panAngle\t\t\t = ", panAngle)
                print("posRobotX\t\t\t = ", posRobotX)
                print("posRobotY\t\t\t = ", posRobotY)
                print("gridTarget\t\t\t = ", gridTarget)
                print("requestPath\t\t\t = ", requestPath)
                print("gridObstacleList\t\t = ", gridObstacleList)
                #print("removeSameList\t\t\t = ", removeSameList)
                print("Grid Send To Main\t\t = ", gridSendToMain)
                print("")
                print("")
                print("")
                #removeSameList                  = gridObstacleList
                #gridObstacleList                = [i for i in gridObstacleList if i not in remove_list]
                #distanceRobotToLandmarks[8] 	= float(strDataFromKinematic[10])	#
                
    			#imuCurrentHeading 		= float(strDataFromKinematic[11])	# Current Heading
    			#imuCurrentHeading 		= float(strDataFromKinematic[9])	# Current Heading
    			#ballDistance 			= float(strDataFromKinematic[12])	#
    			#panAngle 			= float(strDataFromKinematic[13])	# Pan Angle (Robot)
    			#posRobotX 			= float(strDataFromKinematic[14])	#Position RObot X From Odometry+initialPos main
    			#posRobotY 			= float(strDataFromKinematic[15])*-1			#Position Robot Y From Odometry+initialPos main
    			#odomHeading 			= float(strDataFromKinematic[16])			#Heading From Odometri
                #print ("DataFromMain(Mentah) = ",strDataFromKinematic)
                
            # Kalau keluar lapangan random posisi robot yg baru
            if simulationMode == True:
                if robotGlobalPosition[0] < 0 or robotGlobalPosition[0] >= fieldLength or robotGlobalPosition[1] < 0 or robotGlobalPosition[1] >= fieldWidth:
                    robotInitialPosition[0] = uniform(0, fieldLength)
                    robotInitialPosition[1] = uniform(0, fieldWidth)
                    robotInitialPosition[2] = uniform(0, 180)
                    robotGlobalPosition[:] = 0
                    robotLocalPosition[:] = 0

            # Simulate robot movement
            robotLocalPosition[0] += realVelocity[0] * deltaTime
            robotLocalPosition[1] += realVelocity[1] * deltaTime
            if headingFromIMU:
                robotLocalPosition[2] = imuCurrentHeading #- imuInitHeading
            if headingFromIMU:
                angle = robotLocalPosition[2]

            # Create matrix rotation
            theta = np.radians(angle)
            c, s = np.cos(theta), np.sin(theta)
            R = np.array(((c,-s), (s, c)))
            npOutMatMul = np.matmul(R, robotLocalPosition[:2]) 
            robotGlobalPosition[0] = npOutMatMul[0] + robotInitialPosition[0]
            robotGlobalPosition[1] = npOutMatMul[1] + robotInitialPosition[1]
            robotGlobalPosition[2] = angle

            # Predict movement of particles
            particlesLocalPosition[:,0] += realVelocity[0] * deltaTime
            particlesLocalPosition[:,1] += realVelocity[1] * deltaTime
            if headingFromIMU:
                particlesLocalPosition[:,2] = imuCurrentHeading #- imuInitHeading
        
            # Simulate noise movement of robot with error stddev = 10
            simulateNoiseMovement = False
            if simulateNoiseMovement == True:
                particlesLocalPosition[:,0] = normal(particlesLocalPosition[:,0], 10)
                particlesLocalPosition[:,1] = normal(particlesLocalPosition[:,1], 10)
                particlesLocalPosition[:,2] = normal(particlesLocalPosition[:,2], 3)

            # Calculate position of particles in global coordinat
            updateParticlesMovement = True
            if updateParticlesMovement == True:
                for i in range (0,totalParticles):
                    particlesLocalPosition[i,2] = particlesLocalPosition[i,2]

                    # Kalau pakai data IMU dianggap tidak ada rotasi 
                    if headingFromIMU:
                        angle = particlesLocalPosition[i,2]
                    # Kalau pakai yaw rate ditambahkan dulu dengan initial position
                    else:
                        angle = particlesInitialPosition[i,2] + particlesLocalPosition[i,2]
                    theta = np.radians(angle)
                    c, s = np.cos(theta), np.sin(theta)
                    R = np.array(((c,-s), (s, c)))
                    npOutMatMul = np.matmul(R, particlesLocalPosition[i,:2]) 
                    particlesGlobalPosition[i,0] = npOutMatMul[0] + particlesInitialPosition[i,0]
                    particlesGlobalPosition[i,1] = npOutMatMul[1] + particlesInitialPosition[i,1]
                    particlesGlobalPosition[i,2] = angle

                    # Jika keluar lapangan random partikel yang baru di sekitar estimate position terakhir
                    if particlesGlobalPosition[i,0] < 0 or particlesGlobalPosition[i,1] < 0 or particlesGlobalPosition[i,0] >= fieldLength or particlesGlobalPosition[i,1] >= fieldWidth:
                        # Cek kalau estimatenya tdk nan atau inf
                        if math.isnan(estimatePosition[0]) or math.isnan(estimatePosition[1]) or math.isnan(estimatePosition[2]) or math.isinf(estimatePosition[0]) or math.isinf(estimatePosition[1]) or math.isinf(estimatePosition[2]):
                            particlesInitialPosition[i,0] = uniform(0, fieldLength)
                            particlesInitialPosition[i,1] = uniform(0, fieldWidth)
                            particlesInitialPosition[i,2] = uniform(0, 3650)
                            particlesGlobalPosition[i,:] = 0
                            particlesLocalPosition[i,:] = 0
                        else:
                            particlesInitialPosition[i,0] = normal(estimatePosition[0], 50)
                            particlesInitialPosition[i,1] = normal(estimatePosition[1], 50)
                            particlesInitialPosition[i,2] = normal(estimatePosition[2], 10)
                            particlesGlobalPosition[i,:] = 0
                            particlesLocalPosition[i,:] = 0

            # Measurement distance between robot and landmarks
            if simulationMode == True:
                for i in range (0,totalLandmarks):
                    distanceRobotToLandmarks[i] = distance.euclidean([robotGlobalPosition[:2]], [landmarksPosition[i]])

            # Resample only when get valid distance data
            if totalDetectLandmarks >= 2 :
                resample = True
            else:
                resample = False

            # Calculate estimate position
            # Jika ada perintah resample
            if resample == True:
                try :
                    # Measurement distance between particles and landmarks
                    for i in range (0, totalParticles):
                        for j in range (0, totalLandmarks):
                            if distanceRobotToLandmarks[j] > 0 :
                                  distanceParticlesToLandmarks[i,j] = distance.euclidean([particlesGlobalPosition[i,:2]], [landmarksPosition[j]])
                            else :
                                  distanceParticlesToLandmarks[i,j] = 0

                    # Calculating weight
                    # Initialize particles weight with 1.00
                    particlesWeight.fill(1.0)
                    for i in range (0, totalParticles):
                        for j in range (0, totalLandmarks):
                            # mean = jarak robot ke landmark
                            # stddev = 5
                            particlesWeight[i] *= scipy.stats.norm.pdf(distanceParticlesToLandmarks[i,j],distanceRobotToLandmarks[j],5)
                    # Normalize weight
                    totalWeight = sum(particlesWeight)
                    for i in range (0, totalParticles):
                        if totalWeight != 0:
                            particlesWeight[i] = particlesWeight[i] / totalWeight
                    
                    estimatePosition[:] = np.average(particlesGlobalPosition, weights=particlesWeight, axis=0)
                    estimateInitialPosition[:] = estimatePosition[:]
                    estimateLocalPosition[:] = 0
                except :
                    pass

            # Jika tidak update estimate position dengan data dari kinematik
            else:
                estimateLocalPosition[0] += realVelocity[0] * deltaTime
                estimateLocalPosition[1] += realVelocity[1] * deltaTime
                if headingFromIMU:
                    estimateLocalPosition[2] = imuCurrentHeading #- imuInitHeading
                if headingFromIMU:
                    angle = estimateLocalPosition[2]
                theta = np.radians(angle)
                c, s = np.cos(theta), np.sin(theta)
                R = np.array(((c,-s), (s, c)))
                npOutMatMul = np.matmul(R, estimateLocalPosition[:2])
                estimatePosition[0] = int(posRobotX+robotInitialPosition[0])
                estimatePosition[1] = int(posRobotY+robotInitialPosition[1])
                estimatePosition[2] = angle

            # Mark as -888 if result infinity or nan
            if math.isnan(estimatePosition[0]) or math.isnan(estimatePosition[1]) or math.isnan(estimatePosition[2]) or math.isinf(estimatePosition[0]) or math.isinf(estimatePosition[1]) or math.isinf(estimatePosition[2]):
                estimatePosition[:] = -888
                ballEstimatePosition[:] = -888
            # random uniform lagi
            else:
                # ini masih dalam koordinat lokal robot
                ballEstimatePosition[0] = ballDistance
                ballEstimatePosition[1] = 0
                # ini nanti ditambahkan sama posisi servo pan
                headHeading = panAngle;
                if headHeading  >= 360:
                    headHeading = headHeading  - 360
                if headHeading  < 0:
                    headHeading  = 360 + headHeading
                theta = np.radians(headHeading)
                c, s = np.cos(theta), np.sin(theta)
                R = np.array(((c,-s), (s, c)))
                npOutMatMul = np.matmul(R, ballEstimatePosition[:2])
                ballEstimatePosition[0] = npOutMatMul[0] + estimatePosition[0]
                ballEstimatePosition[1] = npOutMatMul[1] + estimatePosition[1]

            # Kirim X, Y, Theta Robot, Ball X, Ball Y
            if simulationMode == False:
                if resample == True:
                    sendToMain()
                    if arahGoal == 0:
                        tempData = str(int(robotCoorToMain[0])) + "," + str(int(robotCoorToMain[1])) + "," + str(int(estimatePosition[2])) + "," + str(int(ballCoorToMain[0])) + "," + str(int(ballCoorToMain[1]))
                        for i in range(len(gridSendToMain)):
                            tempData = tempData + "," + str(gridSendToMain[i])
                        #print(tempData)
                        msgToMainProgram = tempData
                        #msgToMainProgram = "{},{},{},{},{}".format(int(robotCoorToMain[0]), int(robotCoorToMain[1]), int(estimatePosition[2]), int(ballCoorToMain[0]), int(ballCoorToMain[1])) 
                    else :
                        msgToMainProgram = "{},{},{},{},{}".format(int(robotCoorToMain[0]*-1), int(robotCoorToMain[1]*-1), int(estimatePosition[2]), int(ballCoorToMain[0]*-1), int(ballCoorToMain[1]*-1)) 

                elif resample == False:
                    sendToMain()
                    if arahGoal == 0:
                        msgToMainProgram = "{},{},{},{},{}".format(int(-1),int(-1),int(estimatePosition[2]), int(ballCoorToMain[0]), int(ballCoorToMain[1]))
                    else :
                        msgToMainProgram = "{},{},{},{},{}".format(int(-1),int(-1),int(estimatePosition[2]), int(ballCoorToMain[0]), int(ballCoorToMain[1])) 
                sock.sendto(msgToMainProgram.encode('utf-8', 'strict'), (MAIN_UDP_IP, MAIN_UDP_OUT_PORT))

            drawParticles = False
            if drawParticles == True:
                for i in range (0, totalParticles):
                    x, y = worldCoorToImageCoor(int(particlesGlobalPosition[i,0]), int(particlesGlobalPosition[i,1]))
                    cv2.circle(mapImage,(x, y), 7, (0,0,255), -1)

            drawSimRobot = False
            if drawSimRobot == True:
                x, y = worldCoorToImageCoor(int(robotGlobalPosition[0]), int(robotGlobalPosition[1]))
                cv2.circle(mapImage,(x, y), 12, (0,255,255), -1)

            drawEstimatePosition = False
            if drawEstimatePosition == True:
                try:
                    x, y = worldCoorToImageCoor(int(estimatePosition[0]), int(estimatePosition[1]))
                    cv2.circle(mapImage,(x, y), 12, (255,0,0), -1)
                except:
                    pass

            drawOdometryPosition = True
            if drawOdometryPosition == True:
                try:
                    odometryPosition[0] = int(posRobotX+robotInitialPosition[0])
                    odometryPosition[1] = int(posRobotY+robotInitialPosition[1]) 
                    x,y = worldCoorToImageCoor(int(odometryPosition[0]),int(odometryPosition[1]))
                    cv2.circle(mapImage,(x, y), 10, (0,128,255), -1)
                except:
                    pass
                            
            drawBallEstimatePosition = True
            if drawBallEstimatePosition == True:
                try:
                    x, y = worldCoorToImageCoor(int(ballEstimatePosition[0]), int(ballEstimatePosition[1]))
                    cv2.circle(mapImage,(x, y), 10, (255,255,0), -1)
                except:
                    pass

            # Put text on display
            textLine = "R%d Velocity : (%.2f, %.2f, %.2f)"%(robotID, realVelocity[0], realVelocity[1], realVelocity[2])
            cv2.putText(mapImage, textLine, (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.LINE_AA) #OpenCV 3.x
            #cv2.putText(mapImage, textLine, (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.CV_AA) #OpenCV 2.x

            textLine = "R{} Local Position : ({}, {}, {})".format(robotID, int(robotLocalPosition[0]), int(robotLocalPosition[1]), int(robotLocalPosition[2]))
            cv2.putText(mapImage, textLine, (10,50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.LINE_AA) #OpenCV 3.x
            #cv2.putText(mapImage, textLine, (10,50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.CV_AA) #OpenCV 2.x

            textLine = "R{} Global Position : ({}, {}, {})".format(robotID, int(robotGlobalPosition[0]), int(robotGlobalPosition[1]), int(robotGlobalPosition[2]))
            cv2.putText(mapImage, textLine, (340,20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.LINE_AA) #OpenCV 3.x
            #cv2.putText(mapImage, textLine, (340,20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.CV_AA) #OpenCV 2.x

            textLine = "R{} Estimate Position : ({}, {}, {})".format(robotID, int(estimatePosition[0]), int(estimatePosition[1]), int(estimatePosition[2]))
            cv2.putText(mapImage, textLine, (340,50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.LINE_AA) #OpenCV 3.x
            #cv2.putText(mapImage, textLine, (340,50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.CV_AA) #OpenCV 2.x

            #Path Planning
            area_rand = startRobot + goalRobot
            
            if requestPath == 1 and requestPath != lastRequestPath:
                searchPath()
                print("\t\t\t    UPDATE PATH ! ! !")
                lastRequestPath = requestPath
            else:  
                lastRequestPath = requestPath
                print("\t\t\tNOT UPDATE PATH ! ! !")
                
            if len(gridObstacleList) == 0:
                print("\t\t\t   NO OBSTACLE")
            else:
                drawObstacle(len(gridObstacleList))
            
            if pathMode == 0:
                if len(pointOutX) > 0:
                    tempPointOut = np.zeros(len(pointOutX))
                    for i in range(len(pointOutX)):
                        try:
                            #label .retry
                            tempPointGrid = convertCoorToGrid(int(pointOutX[i]),int(pointOutY[i]))
                            tempPointOut[i] = tempPointGrid
                            gridSendToMain = list(dict.fromkeys(tempPointOut.tolist()))
                            gridSendToMain = [i for i in gridSendToMain if i not in gridAfter]
                            #xPath, yPath = convertGridToCoor(tempPointGrid)
                            #xPath, yPath = convertGridToCoor(int(gridSendToMain[i]))
                            #x, y = worldCoorToImageCoor(int(pointOutX[i]),int(pointOutY[i]))
                            #x, y = worldCoorToImageCoor(xPath,yPath)
                            #cv2.circle(mapImage,(x, y), 10, (100,100,0), -1)
                            #print("pointCoordinate\t(X Y) \t%d\t= (%.2f %.2f)"%(i, x, y))
                        except:
                            #print("")
                            #print("\t\tRETRY!!!!!!!!!!")
                            #print("")
                            pass
                            #goto .retry
            
            gridSendToMain = np.trim_zeros(gridSendToMain)
            gridSendToMain = [round(x) for x in gridSendToMain]
            #print(xEarrow)
            #print(yEarrow)
            #cv2.arrowedLine(mapImage, (50, 50), (100, 50), (0, 0, 255), 13, 8, 0, 0.3)
            #Draw Line Path
            for i in range(len(gridSendToMain)):
                xPath, yPath = convertGridToCoor(int(gridSendToMain[i]))
                x, y = worldCoorToImageCoor(xPath,yPath)
                cv2.circle(mapImage,(x, y), 10, (100,100,0), -1)
                if i > 0:
                    xPSL, yPSL = convertGridToCoor(int(gridSendToMain[i-1]))
                    xPEL, yPEL = convertGridToCoor(int(gridSendToMain[i]))
                    xSP , ySP = worldCoorToImageCoor(xPSL, yPSL)
                    xEP , yEP = worldCoorToImageCoor(xPEL, yPEL)
                    #cv2.line(mapImage, (xSP,ySP), (xEP,yEP), (100,100,0), 3)
                    cv2.arrowedLine(mapImage, (xSP, ySP), (xEP, yEP), (100, 100, 0), 8, 8, 0, 0.1)
            drawRobotAndTarget()
            #cv2.arrowedLine(mapImage, (xSDraw, ySDraw), (xEDraw, yEDraw), 0, 255, 0, 1)
            #gridAfter = [round(x) for x in gridAfter]
            #gridSendToMain = [round(x) for x in gridSendToMain]
            #print("Grid Send To Main???\t\t = ", gridSendToMain)
            #gridSendToMain = [i for i in gridSendToMain if i not in gridAfter]
            #print("Grid Send To Main!!!\t\t = ", gridSendToMain)
            
            # Enable GUI Streaming
            showGUI = True
            if showGUI == True:
                smallMapImage = cv2.resize(mapImage, (640,480), interpolation = cv2.INTER_AREA)
                cv2.imshow("Barelang Localization", smallMapImage)

            # Enable URL Streaming
            streamUrl = False
            if streamUrl == True:
                smallMapImage = cv2.resize(mapImage, None, fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)
                cv2.imwrite('output/localization.jpg', smallMapImage)
                
            # Resample
            if resample == True:
                """
                indexHighestWeight = np.argmax(particlesWeight)
                xHighest = particlesGlobalPosition[indexHighestWeight,0]
                yHighest = particlesGlobalPosition[indexHighestWeight,1]
                thetaHighest = particlesGlobalPosition[indexHighestWeight,2]
                """

                _10PercentParticle = int(totalParticles * 0.1)

                for i in range (0, _10PercentParticle):
                    particlesInitialPosition[i,0] = uniform(0, fieldLength)
                    particlesInitialPosition[i,1] = uniform(0, fieldWidth)
                    particlesInitialPosition[i,2] = uniform(0, 360)
                    particlesGlobalPosition[i,:] = 0
                    particlesLocalPosition[i,:] = 0

                _90PercentParticle = totalParticles - _10PercentParticle

                for i in range (_10PercentParticle + 1, _90PercentParticle):
                    if math.isnan(estimatePosition[0]) or math.isnan(estimatePosition[1]) or math.isnan(estimatePosition[2]) or math.isinf(estimatePosition[0]) or math.isinf(estimatePosition[1]) or math.isinf(estimatePosition[2]):
                        particlesInitialPosition[i,0] = uniform(0, fieldLength)
                        particlesInitialPosition[i,1] = uniform(0, fieldWidth)
                        particlesInitialPosition[i,2] = uniform(0, 360)
                        particlesGlobalPosition[i,:] = 0
                        particlesLocalPosition[i,:] = 0
                    else:
                        particlesInitialPosition[i,0] = normal(estimatePosition[0], 50)
                        particlesInitialPosition[i,1] = normal(estimatePosition[1], 50)
                        particlesInitialPosition[i,2] = normal(estimatePosition[2], 10)
                        particlesGlobalPosition[i,:] = 0
                        particlesLocalPosition[i,:] = 0

            if showGUI:
                #cv2.waitKey(int(deltaTime*1000))
                key = cv2.waitKey(1)
                if key == 27:
                    break
							
if __name__ == "__main__":
        print ('Running BarelangFC - MCL Localization')
        main()

