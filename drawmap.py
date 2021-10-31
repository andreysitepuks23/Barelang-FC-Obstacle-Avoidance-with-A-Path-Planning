import time
import numpy as np
import cv2
import math
from os import system

# Particles and landmarks
deltaTime = 1

mapImage = np.zeros((800,1100,3), np.uint8)

# Global Variable
velFromKinematic = np.zeros((3))

currentPosX = 0 #start Coor X
currentPosY = 0 #start Coor X
currentA = 0
targetX = 0 #target coor X
targetY = 0 #target coor Y
lastTargetX = 0 #target coor X
lastTargetY = 0 #target coor Y
offsetRange = 8
angle = -170
robotSpeed = np.zeros((3))
isRotate = True
#listPath = [9,21,26,32,38,45,40,35,29,23,18,11,10]
listPath = [15,20,26,32,39,40,35,29,23,16,9,8,13,19,25,31,38,45,46,41,36,30,24,17,10]
#listPath.reverse()
#listPath = [38,54]
listPathLen = 0
isArrive = False
pathCount = 0
textLabel = "NONE"
moveLabel = "--__--"
sPointXMax = 0
sPointXMin = 0
sPointYMax = 0
sPointYMin = 0
loopPath = 0

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
            
    elif robotId == 3:				    #JIKA ROBOT 1
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

def worldCoorToImageCoor(x, y):
    x = x + 100
    y = 800 - (y + 100)
    return x, y

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

def calculateCoorWithAngle(xSA, ySA, a, l):
    thetaAngle = np.radians(a)
    cosAngle, sinAngle = np.cos(thetaAngle), np.sin(thetaAngle)
    resultX = int(xSA + (l * cosAngle))
    resultY = int(ySA + (l * sinAngle))
    
    return resultX, resultY

def jalanDirection(Xwalk,Ywalk,rotate):
    global angle
    global isRotate
    global robotSpeed
    
    if angle > 180:
        selisih = abs(angle - 180)
        angle = -180 + (selisih)
    elif angle < -180:
        angle = 360 + angle
        
    if rotate > 180:
        rotate = 180
    elif rotate < -180:
        rotate = 180
    setPoint1 =  5 + rotate
    setPoint2 = -5 + rotate
    #print(setPoint1)
    
    if setPoint1 > 180 or setPoint2 < -180: #jika arah target dibelakang
        if setPoint1 > 180: #nilai setpoint1 diubah jd negatif
            btsSetPoint = setPoint1 - 360
            if angle >= setPoint1 or angle <= btsSetPoint: #misal 170 ke -170
                PaMove = 0.00
                isRotate = False
            else:
                btsRotate = rotate - 180
                if angle <= rotate and angle >= btsRotate: #misal di range 0 - 180, maka putar kanan
                    putar = (rotate - angle) * 0.0065
                    PaMove = -putar
                    #print(PaMove)
                    #print("CROOOOOOOOOOOOOOOOOOOOOOOT")
                    if PaMove <= -0.009:
                        angle = rotate
                        isRotate = False
                else: #putar kiri
                    if angle > rotate:
                        putar = (angle - rotate) * 0.004
                    else:
                        putar = ((180-rotate) + (180 + angle)) * 0.004
                    PaMove = putar
        else: #nilai setPoint2 diubah jadi positif
            btsSetPoint = setPoint2 + 360
            if angle >= btsSetPoint or angle <= setPoint1:
                PaMove = 0.00
                #print(PaMove)
                #print("CROOOOOOOOOOOOOOOOOOOOOOOT")
                if PaMove == 0.0:
                    angle = rotate
                    isRotate = False
                isRotate = False
            else:
                btsRotate = rotate + 180
                if angle >= rotate and angle <= btsRotate: #misal di range -180 - 0, maka putar kiri
                    putar = abs(rotate - angle) * 0.004
                    PaMove = putar
                else: #putar kanan
                    if angle < rotate:
                        putar = (rotate - angle) * 0.004
                    else:
                        putar = ((180 - rotate) + (180 - angle)) * 0.004
    else: #arah target kedepan
        if angle >= setPoint2 and angle <= setPoint1:
            PaMove = 0.00
            isRotate = False
        else:
            if rotate >= 0:
                btsRotate = rotate - 180
                if angle <= rotate and angle >= btsRotate: #putar kanan
                    putar = (rotate - angle) * 0.004
                    PaMove = -putar
                    #isRotate = False
                else: #putar kiri
                    if angle > rotate:
                        putar = (angle - rotate) * 0.004
                    else:
                        putar = ((180 - rotate) + (180 + angle)) * 0.004
                    PaMove = putar
            else:
                btsRotate = rotate + 180
                if angle >= rotate and angle <= btsRotate: #maka putar kiri
                    putar = abs(rotate - angle) * 0.004
                    PaMove = putar
                else: #putar kanan
                    if angle < rotate:
                        putar = (rotate - angle) * 0.004
                    else:
                        putar = ((180 + rotate) + (180 - angle)) * 0.004
                    PaMove = -putar                 
    if PaMove > 0.3:
        PaMove = 0.3
    elif PaMove < -0.3:
        PaMove = -0.3
    
    velFromKinematic[2] = PaMove
    robotSpeed = convertVel(1, velFromKinematic)
    angle = angle - robotSpeed[2] 
    
def updateMove(targetA):
    global currentA
    global currentPosX
    global currentPosY
    global targetX
    global targetY
    global robotSpeed
    global isRotate
    global moveLabel
    #print(targetX)
    #print(targetY)
    if isRotate == False:
        if targetA >= 175 or targetA <= -175:
            #print("180")
            if targetA > 0:
                #print("positif")
                if (angle >= 175) or (angle <= -175):
                    currentPosX, currentPosY = calculateCoorWithAngle(currentPosX, currentPosY, targetA, robotSpeed[0])
                    #print("Move X")
                    moveLabel = "X Move"
                else:
                    isRotate = True
            else:
                #print("negatif")
                if (angle <= -175) or (angle >= 175):
                    currentPosX, currentPosY = calculateCoorWithAngle(currentPosX, currentPosY, targetA, robotSpeed[0])
                    #print("Move X")
                    moveLabel = "X Move"
                else:
                    isRotate = True
        else:
            if (angle >= targetA-5) and (angle <= targetA+5):
                currentPosX, currentPosY = calculateCoorWithAngle(currentPosX, currentPosY, targetA, robotSpeed[0])
                #print("Move X")
                moveLabel = "X Move"
            else:
                isRotate = True
    else:
        jalanDirection(velFromKinematic[0], velFromKinematic[1], targetA)
        #print("Rotate")
        moveLabel = "Rotate Move"
        
def drawFromCenterText(x, y, text):
    textSize = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0]
    textX = int((x - textSize[0]/2))
    textY = int((y + textSize[1]/2))
    textX, textY = worldCoorToImageCoor(textX,textY)
    cv2.putText(mapImage, text, (textX, textY), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
 
def main():
    global currentA
    global currentPosX
    global currentPosY
    global targetX
    global targetY
    global pathCount
    global isArrive
    global lastTargetX
    global lastTargetY
    global textLabel
    global robotSpeed
    global loopPath
    
    currentPosX = 150
    currentPosY = 600
    targetGrid = 7
    targetX, targetY = convertGridToCoor(targetGrid)
    targetX, targetY = worldCoorToImageCoor(targetX,targetY)
    sPointXMax = targetX + offsetRange
    sPointXMin = targetX - offsetRange
    sPointYMax = targetY + offsetRange
    sPointYMin = targetY - offsetRange
    currentPosX, currentPosY = worldCoorToImageCoor(currentPosX,currentPosY)
    listPathLen = len(listPath)
    # Set initial location of robot, Just for simulation 
    velFromKinematic[0] = 0.03
    velFromKinematic[1] = 0.00
    velFromKinematic[2] = 0.00
    
    # Timing value
    nowTime = 0
    lastTime = 0
    loop = 0

    while True:
        nowTime = time.clock()
        timer = nowTime - lastTime
        halfDeltaTime = deltaTime / 2.00
        # Update every 0.5 * deltatime
        if timer > halfDeltaTime:
            system('cls')
            lastTime = nowTime
            loop += 1
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
                    
            for i in range(len(listPath)):
                if i > 0:
                    xPSL, yPSL = convertGridToCoor(int(listPath[i-1]))
                    xPEL, yPEL = convertGridToCoor(int(listPath[i]))
                    xSP , ySP = worldCoorToImageCoor(xPSL, yPSL)
                    xEP , yEP = worldCoorToImageCoor(xPEL, yPEL)
                    cv2.line(mapImage, (xSP,ySP), (xEP,yEP), (255,0,127), 1)
                    if i == listPathLen-1:
                        xPSL, yPSL = convertGridToCoor(int(listPath[0]))
                        xSP , ySP = worldCoorToImageCoor(xPSL, yPSL)
                        cv2.line(mapImage, (xSP,ySP), (xEP,yEP), (255,0,127), 1)

            if isArrive == False:
                #print("Go To Target")
                textLabel = "Reaching Target"
                #print(currentPosX - sPointXMin)
                if abs(currentPosX - sPointXMin) <= offsetRange or abs(currentPosX - sPointXMax) <= offsetRange:
                    if abs(currentPosY - sPointYMin) <= offsetRange or abs(currentPosY - sPointYMax) <= offsetRange:
                        #print("Target Reached")
                        textLabel = "Target Reached"
                        lastTargetX = targetX
                        lastTargetY = targetY
                        targetGrid = listPath[pathCount]
                        targetX, targetY = convertGridToCoor(targetGrid)
                        targetX, targetY = worldCoorToImageCoor(targetX,targetY)
                        sPointXMax = targetX + offsetRange
                        sPointXMin = targetX - offsetRange
                        sPointYMax = targetY + offsetRange
                        sPointYMin = targetY - offsetRange
                        pathCount += 1
                        if pathCount > listPathLen - 1:
                            pathCount = 0
                            loopPath += 1
                        #print("Next Target")
                        isArrive = True
            else:
                #print("Tunggu")
                if abs(currentPosX - lastTargetX) >= offsetRange or abs(currentPosY - lastTargetX) >= offsetRange:
                    isArrive = False
                    #print("Go To Next Target")
                    textLabel = "Next Target"
            
            #speed = convertVel(1,velFromKinematic)           
            targetRad = math.atan2(targetY-currentPosY, targetX-currentPosX)
            targetDegree = math.degrees(targetRad)          
            updateMove(targetDegree)           
            xAngleRobot, yAngleRobot = calculateCoorWithAngle(currentPosX, currentPosY, angle, 25)
            cv2.line(mapImage, (currentPosX,currentPosY), (targetX,targetY), (127,0,127), 2)
            cv2.arrowedLine(mapImage, (currentPosX, currentPosY), (xAngleRobot, yAngleRobot), (0, 127, 255), 10, 8, 0, 1)
            cv2.circle(mapImage,(currentPosX, currentPosY), 15, (0,0,255), -1)
            cv2.circle(mapImage,(targetX, targetY), 15, (255,0,0), -1)
            drawFromCenterText(450, 660, textLabel)
            drawFromCenterText(450, 620, moveLabel)
            
            #print Data
            print("Path Length : ",listPathLen)
            print("Path Counter : ",pathCount+1)
            print("Robot Speed X :",robotSpeed[0])
            print("Robot Speed Y :",robotSpeed[1])
            print("Robot Speed A :",robotSpeed[2])
            print("Current Robot X : ",currentPosX)
            print("Current Robot Y : ",currentPosY)
            print("Target Robot X : ",targetX)
            print("Target Robot Y : ",targetY)
            print("Target Angle : ",targetDegree)
            print("Current Angle : ",angle)
            print("Rotate State : ", isRotate)
            print("Arrive State : ", isArrive)
            print("LoopPath : ",loopPath)

            print(" ")
            # Enable GUI Streaming
            showGUI = True
            if showGUI == True:
                smallMapImage = cv2.resize(mapImage, (640,480), interpolation = cv2.INTER_AREA)
                cv2.imshow("Robot Move Simulator", smallMapImage)
            
            if showGUI:
                #cv2.waitKey(int(deltaTime*1000))
                key = cv2.waitKey(1)
                if key == 27:
                    break
if __name__ == "__main__":
        print ('Running BarelangFC - Robot Move Simulator')
        main()
