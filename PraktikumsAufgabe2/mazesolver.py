import time
from math import *
from Robot_Simulator_V3 import labyrinthWorld, geometry, emptyWorld
from Robot_Simulator_V3 import Robot
import numpy as np

from Robot_Simulator_V3.sensorUtilities import extractSegmentsFromSensorData, transformPolylinesL2G

myWorld = labyrinthWorld.buildWorld()
#myWorld = emptyWorld.buildWorld()
myRobot = Robot.Robot()
p1 = (2, 18)
myWorld.setRobot(myRobot, [p1[0], p1[1], pi / 2])


myRobot._k_d = 0.05*0.0  # velocity noise parameter = 0.05m*0.05m / 1m
myRobot._k_theta = 0.05*0.0  # turning rate noise parameter = 5deg*5deg/360deg * (1rad/1deg)
myRobot._k_drift = 0.05*0.0


def straightDrive(v, l):
    n = int((l / v) * 10)
    # Bewege Roboter
    for t in range(n):
        # Bewege Roboter
        motion = (v, 0)
        myRobot.move(motion)

def curveDrive(v, r, theta):
    l = (abs(theta) * 2 * pi / 360) * r # laenge des kreisabschnitts
    t = int((l / v) * 10) # zeit benoetigt fuer die strecke

    for n in range(t):
        motion = (v, (theta * 2 * pi / 360)/ (t/10))
        myRobot.move(motion)


def wander(v):
    numBeams= myRobot._numberOfBeams
    angle = myRobot._viewAngle
    degree = angle / numBeams

    while(True):
        pos = -1
        min = 99999
        senor = myRobot.sense()
        for i in range(0, numBeams):
            if(senor[i] != None and senor[i] < 1):
                if(min > senor[i]):
                    min = senor[i]
                    pos = i
        #print(pos)

        if(pos == -1):
            th = 0
        else:
            break
            th = ((pos * degree + 180 - 135 + 180) % 360) - 180
            th = (th * 2 * pi) / 360

        x, y, theta = myWorld.getTrueRobotPose()
        wallpoints = extractSegmentsFromSensorData(myRobot.sense(), myRobot.getSensorDirections())
        myWorld.drawPolyline([(wallpoints[0][0][0] + x, wallpoints[0][0][1] + y), (wallpoints[1][1][0] + x, wallpoints[1][1][1] + y)])
        myRobot.move((v, th))


def shortest(v, d):
    numBeams = myRobot._numberOfBeams
    angle = myRobot._viewAngle
    degree = angle / numBeams
    old_th = 0
    while (True):
        pos = -1
        min = 99999
        senor = myRobot.sense()
        for i in range(0, int(numBeams/2)):
            if (senor[i] != None):
                if (min > senor[i]):
                    min = senor[i]
                    pos = i

        if (pos == -1):
            print("pos -1")
            th = old_th
        else:
            th = pos * degree - 135 + 90
            if (senor[pos] < d):
                th = th + 5
            th = (th * 2 * pi) / 360
            if(senor[pos] > 1.5):
                print(pos, senor[pos], old_th)
                th = old_th

            old_th = th
        myRobot.move((v, th*10*v))



def followWall(v, d):
    wander(0.1)
    a = 0

    x,y, theta = myWorld.getTrueRobotPose()

    wallpoints = extractSegmentsFromSensorData(myRobot.sense(), myRobot.getSensorDirections())
    print(wallpoints)

    polarpointX1 = np.sqrt(wallpoints[0][0][0]**2 + wallpoints[0][0][1]**2) * np.cos(np.arctan2(wallpoints[0][0][1],wallpoints[0][0][0]))
    polarpointY1 = np.sqrt(wallpoints[0][0][0]**2 + wallpoints[0][0][1]**2) * np.sin(np.arctan2(wallpoints[0][0][1],wallpoints[0][0][0]))
    polarpointX2 = np.sqrt(wallpoints[0][1][0]**2 + wallpoints[0][1][1]**2) * np.cos(np.arctan2(wallpoints[0][1][1],wallpoints[0][1][0]))
    polarpointY2 = np.sqrt(wallpoints[0][1][0]**2 + wallpoints[0][1][1]**2) * np.sin(np.arctan2(wallpoints[0][1][1],wallpoints[0][1][0]))
    #print(polarpointX1, polarpointY1)

    myWorld.drawPolyline([(polarpointX1 + x , polarpointY1 + y), (polarpointX2 + x, polarpointY2 + y)])

    transformtWallpoints = transformPolylinesL2G(wallpoints, myWorld.getTrueRobotPose())
    npx, npy = geometry.neareastPointOnLine((0,0), [(polarpointX1, polarpointY1), (polarpointX2, polarpointY2)])
    print(npx + x, npy + y)
    roto = np.arctan2(npy, npx)
    myRobot.move((v, roto))

    r = np.sqrt(npx**2 + npy**2)

    while (r > 0.9):
        x, y, theta = myWorld.getTrueRobotPose()
        wallpoints = extractSegmentsFromSensorData(myRobot.sense(), myRobot.getSensorDirections())

        polarpointX1 = np.sqrt(wallpoints[0][0][0] ** 2 + wallpoints[0][0][1] ** 2) * np.cos(
            np.arctan2(wallpoints[0][0][1], wallpoints[0][0][0]))
        polarpointY1 = np.sqrt(wallpoints[0][0][0] ** 2 + wallpoints[0][0][1] ** 2) * np.sin(
            np.arctan2(wallpoints[0][0][1], wallpoints[0][0][0]))

        polarpointX2 = np.sqrt(wallpoints[0][1][0] ** 2 + wallpoints[0][1][1] ** 2) * np.cos(
            np.arctan2(wallpoints[0][1][1], wallpoints[0][1][0]))
        polarpointY2 = np.sqrt(wallpoints[0][1][0] ** 2 + wallpoints[0][1][1] ** 2) * np.sin(
            np.arctan2(wallpoints[0][1][1], wallpoints[0][1][0]))
        # print(polarpointX1, polarpointY1)
        myWorld.drawPolyline([(polarpointX1 + x, polarpointY1 + y), (polarpointX2 + x, polarpointY2 + y)])

        npx, npy = geometry.neareastPointOnLine((0, 0), [(polarpointX1, polarpointY1), (polarpointX2, polarpointY2)])
        r = np.sqrt(npx ** 2 + npy ** 2)
        myRobot.move((v, 0))
        #time.sleep(20000)
shortest(0.5, 1)
#followWall(0.1,4)
curveDrive(0.1,0.5,-90)
straightDrive(1, 2)
shortest(0.1)
time.sleep(22220)
#myRobot.move((1, 0))
#followWall(0.1,4)
#
#wander(0.1)