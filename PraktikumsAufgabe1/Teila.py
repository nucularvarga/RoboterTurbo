from math import *
from Robot_Simulator_V3 import emptyWorld
from Robot_Simulator_V3 import Robot
import numpy as np

# Roboter in einer Welt positionieren:
myWorld = emptyWorld.buildWorld()
myRobot = Robot.Robot()
p1 = (2, 5.5)
myWorld.setRobot(myRobot, [p1[0], p1[1], pi / 2])

myRobot._k_d = 0.05*0.0  # velocity noise parameter = 0.05m*0.05m / 1m
myRobot._k_theta = 0.05*0.0  # turning rate noise parameter = 5deg*5deg/360deg * (1rad/1deg)
myRobot._k_drift = 0.05*0.0

# Anzahl Zeitschritte n mit jeweils der Laenge T = 0.1 sec definieren.
# T laesst sich ueber die Methode myRobot.setTimeStep(T) einstellen.
# T = 0.1 sec ist voreingestellt.

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

#straightDrive(1, 2)
#curveDrive(0.01, 0.01, -90)

#############################################################################
#Teil b
#y = mx + b
#m = (y2 - y1)/(x2 - x1)

def followline(p1, p2, tol):
    v = 1
    #myWorld.addLine(p1[0]+1, p1[1]+1, p2[0]+1, p2[1]+1) #draw line to follow
    #y = m*x + b
    #m
    #T = 0.1s
    XisCONST = False
    if(p2[0] - p1[0] == 0):
        XisCONST = True
        gotoGlobal(0.1, (-p2[0],-p2[1]), tol)
    else:
        m = (p2[1] - p1[1]) / (p2[0] - p1[0])
        b = p2[1] - (m * p2[0])

    while (not (abs(myWorld.getTrueRobotPose()[0] - p2[0]) <= tol and abs(myWorld.getTrueRobotPose()[1] - p2[1]) <= tol)):
        x, y, theta = myWorld.getTrueRobotPose()
        if(not XisCONST):
            if(p2[0] >= p1[0]):
                nextX = x + 0.1
            else:
                nextX = x - 0.1
            nextY = m * nextX + b
            rotate = np.arctan2((nextY - y), (nextX - x))
            rotate = (rotate - theta) *180 / pi
        else:
            h = 0.2
            yh = sqrt((x-p1[0])**2 + (h)**2)
            if(theta <= pi/2 and theta >= - pi/2):
                rotate = 90 - ((np.arcsin(h/yh) * 180) / pi)
            else:
                rotate = -(90 - ((np.arcsin(h / yh) * 180) / pi))
            #rotate = 90 - (theta  * 180) / pi
            print("x konstant", np.arcsin(h/yh) * 180 / pi, rotate, theta * 180 / pi)

        if(rotate < 0):
            rotate = -(abs(rotate)%360)
        else:
            rotate = rotate % 360

        if(rotate > 180):
            rotate = rotate - 360
        elif(rotate < -180):
            rotate = rotate + 360

        myRobot.move((0.1, rotate/100))



def gotoGlobal(v, p ,tol):
    x, y, theta = myWorld.getTrueRobotPose()
    rotate = np.arctan2((p[1] - y), (p[0] - x))
    print(x,y,theta*180/pi, rotate*180/pi)
    if(rotate*180/pi < -90):
        rotate = pi + rotate
        theta = 0
    curveDrive(0.01, 0.01, (rotate)*180/pi )
    curveDrive(0.01, 0.01, (-theta) * 180 / pi)
    while(not (abs(myWorld.getTrueRobotPose()[0] - p[0]) <= tol and abs(myWorld.getTrueRobotPose()[1] - p[1]) <= tol)):
        myRobot.move((v, 0))





def followPolyline(v, poly):
    gotoGlobal(v, (poly[0][0], poly[0][1]), 0.5)
    for n in range(len(poly) - 1):
        followline((poly[n][0], poly[n][1]), (poly[n+1][0], poly[n+1][1]), 0.5)

#p1 is spawnposition of robot

polylinoses = [(3, 3),(5,5), (5,15)]
myWorld.drawPolyline(polylinoses)
followPolyline(0.1, polylinoses)


#p2 = (9, 9) # (x, y)
#followline(p1, p2, 0.1)

#gotoGlobal(0.1, (8,9), 0.5)

# Simulation schliessen:
myWorld.close()