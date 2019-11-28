#!/usr/bin/env python

# Some suitable functions and data structures for drawing a map and particles

import time
import random
import math
import sys
import statistics
import numpy as np
import brickpi333 as brickpi3

# PARTICLES STUFF
E_ERROR = 2
F_ERROR = 0.001
G_ERROR = 0.04
VARIANCE = 10
ROBUSTNESS = 0
SONAR_OFFSET = 8.2
PARTICLES = []
SCALING_FACTOR = 1

BP = brickpi3.BrickPi333()
# Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
BP.set_sensor_type(BP.PORT_1, BP.SENSOR_TYPE.NXT_ULTRASONIC)
BP.set_sensor_type(BP.PORT_2, BP.SENSOR_TYPE.TOUCH)
BP.set_sensor_type(BP.PORT_3, BP.SENSOR_TYPE.TOUCH)
####################################################################


def objectFound():
    sonarReading = getSonarReading()
    thisXCoord = getXCoord()
    thisYCoord = getYCoord()
    thisAngle = getAngle()
    print("...Angle now: ", getAngleInDeg())
    D = calculateDistanceFromWall(thisXCoord, thisYCoord, thisAngle)
    print("Expected reading: ", D)
    print("Sonar reading: ", sonarReading)
    if (sonarReading / D < 0.8) and (sonarReading < 100):
        print("...Found Something Interesting!")
        print("...It's so far far away, exactly ", sonarReading, "cm away.")
        #goToCoord(thisXCoord + (sonarReading) * math.cos(thisAngle) - 5,
        #          thisYCoord + (sonarReading) * math.sin(thisAngle) - 5)
        print("...Right in front of the object now.")
        #turn(math.pi / 2)
        #goToCoord(thisXCoord, thisYCoord)
        return True
    return False


def wallIntersects(wall, x, y, theta):
    y1 = min(wall[1], wall[3])
    y2 = max(wall[1], wall[3])
    x1 = min(wall[0], wall[2])
    x2 = max(wall[0], wall[2])
    xintersect = 0
    yintersect = 0
    if (y1 == y2):
        # Horizontal Wall
        if (theta % math.pi == 0):
            return -1
        z = (y1 - y) / (math.sin(theta))
        xintersect = x + z * math.cos(theta)
        if (x1 > xintersect or xintersect > x2):
            return -1
        return z
    else:
        # Vertical Wall
        z = (x1 - x) / (math.cos(theta))
        yintersect = y + z * math.sin(theta)
        if (y1 > yintersect or yintersect > y2):
            return -1
        return z


def calculateDistanceFromWall(x, y, theta):
    if (theta / (math.pi / 2) % 2 == 1):
        # Robot is facing vertical
        minDistance = 1000
        for wall in mymap.walls:
            if (wall[1] == wall[3] and wall[0] < x and x < wall[2]):
                minDistance = abs(y - wall[1])
        return minDistance

    minDistance = 10000
    for wall in mymap.walls:
        z = wallIntersects(wall, x, y, theta)
        if (z >= 0 and z < minDistance):
            minDistance = z
    return minDistance


#####################################################################

def calculateLikelihoodForAllParticles(z):
    global PARTICLES
    for p in PARTICLES.data:
        weight = calculateLikelihood(p[0], p[1], p[2], z)
        p = (p[0], p[1], p[2], weight)


def calculateLikelihood(x, y, theta, z):
    global VARIANCE
    m = calculateDistanceFromWall(x, y, theta)
    error = z - m
    return math.e ** (-((error) ** 2) / 2 * VARIANCE) + ROBUSTNESS


def normalise():
    global PARTICLES
    particlesData = PARTICLES.data
    sum = 0
    for p in particlesData:
        sum += p[3]
    for p in particlesData:
        p = (p[0], p[1], p[2], p[3] / sum)


def updateParticlesAfterRotation(angle):
    global G_ERROR, PARTICLES
    particlesData = PARTICLES.data
    for i in range(len(particlesData)):
        g = random.gauss(0, G_ERROR)

        x = particlesData[i][0]
        y = particlesData[i][1]
        theta = particlesData[i][2]
        weight = particlesData[i][3]

        particlesData[i] = (x, y, theta + (angle) + g, weight)
    MCL()


def updateParticlesAfterMoving(D):
    global E_ERROR, F_ERROR, SCALING_FACTOR, PARTICLES
    particlesData = PARTICLES.data
    for i in range(len(particlesData)):
        e = random.gauss(0, E_ERROR)
        f = random.gauss(0, F_ERROR)

        x = particlesData[i][0]
        y = particlesData[i][1]
        theta = particlesData[i][2]
        weight = particlesData[i][3]

        particlesData[i] = (
            x + (D + e) * math.cos(theta) * SCALING_FACTOR,
            y + (D + e) * math.sin(theta) * SCALING_FACTOR,
            theta + f,
            weight,
        )
    MCL()


def MCL():
    global PARTICLES
    # PARTICLES.draw()
    sonarReading = getSonarReading()
    calculateLikelihoodForAllParticles(sonarReading)
    normalise()
    resample()


def resample():
    global PARTICLES
    particleData = PARTICLES.data
    newParticles = []
    for p in particleData:
        randomValue = random.gauss(0, 1)
        sampleFrom = p
        found = False
        for pp in particleData:
            randomValue -= pp[3]
            if not found and randomValue < 0:
                sampleFrom = pp
                found = True
                continue
        newParticles.append(sampleFrom)
    PARTICLES.data = newParticles


def getXCoord():
    global PARTICLES
    sum = 0
    for i in range(len(PARTICLES.data)):
        sum += PARTICLES.data[i][0]
    avg = sum / len(PARTICLES.data)
    return avg


def getYCoord():
    global PARTICLES
    sum = 0
    for i in range(len(PARTICLES.data)):
        sum += PARTICLES.data[i][1]
    avg = sum / len(PARTICLES.data)
    return avg


def getAngle():
    global PARTICLES
    sum = 0
    for i in range(len(PARTICLES.data)):
        sum += PARTICLES.data[i][2]
    avg = sum / len(PARTICLES.data)
    return avg

def getAngleInDeg():
    return math.degrees(getAngle())

#####################################################################


def moveForwardUntilHitAndReverse():
    start_posi_a = BP.get_motor_encoder(BP.PORT_A)
    start_posi_d = BP.get_motor_encoder(BP.PORT_D)
    BP.set_motor_power(BP.PORT_A, 30)
    BP.set_motor_power(BP.PORT_D, 30)
    hitObject = False
    while not hitObject:
        # if hit object, stops
        if abs(BP.get_motor_encoder(BP.PORT_D)) % 8 == 0:
            BP.set_motor_power(BP.PORT_D, 28)
        elif abs(BP.get_motor_encoder(BP.PORT_D)) % 4 == 0:
            BP.set_motor_power(BP.PORT_D, 30)
        touch1 = BP.get_sensor(BP.PORT_2)
        touch2 = BP.get_sensor(BP.PORT_3)
        if touch1 == 1 or touch2 == 1:
            hitObject = True
    stopMoving()
    atOriginalPosition = False
    BP.set_motor_power(BP.PORT_A, -30)
    BP.set_motor_power(BP.PORT_D, -30)
    while not atOriginalPosition:
        if abs(BP.get_motor_encoder(BP.PORT_D)) % 8 == 0:
            BP.set_motor_power(BP.PORT_D, -28)
        elif abs(BP.get_motor_encoder(BP.PORT_D)) % 4 == 0:
            BP.set_motor_power(BP.PORT_D, -30)
        motor_A = BP.get_motor_encoder(BP.PORT_A)
        motor_D = BP.get_motor_encoder(BP.PORT_D)
        if motor_A <= start_posi_a or motor_D <= start_posi_d:
            atOriginalPosition = True
    stopMoving()


def goToCoord(newXCoord, newYCoord):
    xCoord = getXCoord()
    yCoord = getYCoord()
    distance = math.sqrt((newXCoord - xCoord) ** 2 + (newYCoord - yCoord) ** 2)
    angle = angleToTurn(xCoord, yCoord, newXCoord, newYCoord)
    turn(angle)
#    print("x: ", xCoord)
#    print("y: ", yCoord)

    while distance > 20:
        moveForward(20)
        xCoord = getXCoord()
        yCoord = getYCoord()
#        print("x: ", xCoord)
#        print("y: ", yCoord)
        distance = math.sqrt((newXCoord - xCoord) ** 2 +
                             (newYCoord - yCoord) ** 2)
        angle = angleToTurn(xCoord, yCoord, newXCoord, newYCoord)
        turn(angle)
    xCoord = getXCoord()
    yCoord = getYCoord()
    distance = math.sqrt((newXCoord - xCoord) ** 2 + (newYCoord - yCoord) ** 2)
    moveForward(distance)


def angleToTurn(xCoord, yCoord, newXCoord, newYCoord):
    newAngle = 0
    if xCoord == newXCoord and yCoord < newYCoord:
        newAngle = math.pi / 2
    elif yCoord == newYCoord and xCoord > newXCoord:
        newAngle = 3 * math.pi / 2
    else:
        newAngle = math.atan((newYCoord - yCoord) / (newXCoord - xCoord))

    if xCoord <= newXCoord and yCoord <= newYCoord:
        newAngle = newAngle
    elif xCoord > newXCoord and yCoord <= newYCoord:
        newAngle = math.pi + newAngle
    elif xCoord > newXCoord and yCoord > newYCoord:
        newAngle = math.pi + newAngle
    elif xCoord <= newXCoord and yCoord > newYCoord:
        newAngle = 2 * math.pi + newAngle
    return newAngle - getAngle()


def moveForward(distance):
    circum = 21
    angleToRotate = distance / circum * 360
    start_posi_a = BP.get_motor_encoder(BP.PORT_A)
    start_posi_d = BP.get_motor_encoder(BP.PORT_D)
    BP.set_motor_power(BP.PORT_A, 30)
    BP.set_motor_power(BP.PORT_D, 30)
    while (
        abs(BP.get_motor_encoder(BP.PORT_D) - start_posi_d)
        + abs(BP.get_motor_encoder(BP.PORT_A) - start_posi_a)
    ) / 2 < angleToRotate:
        if abs(BP.get_motor_encoder(BP.PORT_D)) % 8 == 0:
            BP.set_motor_power(BP.PORT_D, 28)
        elif abs(BP.get_motor_encoder(BP.PORT_D)) % 4 == 0:
            BP.set_motor_power(BP.PORT_D, 30)
    stopMoving()
    updateParticlesAfterMoving(distance)


def turn(angle):
    angle = angle % (2 * math.pi)
    start_posi_d = BP.get_motor_encoder(BP.PORT_D)
    start_posi_a = BP.get_motor_encoder(BP.PORT_A)
    if angle < math.pi:
        BP.set_motor_power(BP.PORT_A, -20)
        BP.set_motor_power(BP.PORT_D, 20)
        angleToTurn = angle
    else:
        BP.set_motor_power(BP.PORT_A, 20)
        BP.set_motor_power(BP.PORT_D, -20)
        angleToTurn = 2 * math.pi - angle
    rotations = angleToTurn * 302 / (math.pi / 2)
    while (
        int(
            (
                abs(BP.get_motor_encoder(BP.PORT_D) - start_posi_d)
                + abs(BP.get_motor_encoder(BP.PORT_A) - start_posi_a)
            )
        )
        < rotations
    ):
        a = 0
    stopMoving()
    print("... Angle is now  ", getAngleInDeg())
    updateParticlesAfterRotation(angle)


def stopMoving():
    BP.set_motor_power(BP.PORT_A, 0)
    BP.set_motor_power(BP.PORT_D, 0)
    time.sleep(1)

#####################################################################
def sweep(area):
    if (area == 0):
        turn(0 - getAngle())
    elif (area == 1):
        turn(math.pi / 2 - getAngle())
    elif (area == 2):
        turn(math.pi / 2- getAngle())

    turn(math.pi * 3 / 2)
    i = 0
    max = 0
    count = 0
    found = True
    while i < 20:
        print("Scanning...")
        turn(math.pi * (1 / 20))
        if (objectFound()):
            print("BOTTLE FOUND!")
            max += getAngle()
            count += 1
            found = False
        elif (not found):
            break
        i += 1
    if (max == 0):
        return False
    turn(max / count - getAngle())
    moveForwardUntilHitAndReverse()
    return True
    #turn(math.pi * 3 / 2)


def getSonarReading():
    sonarSum = 0
    for i in range(3):
        sonarSum += BP.get_sensor(BP.PORT_1) + SONAR_OFFSET
    return sonarSum / 3

#####################################################################


def calcX():
    return random.gauss(80, 3) + 70*(math.sin(t))  # in cm


def calcY():
    return random.gauss(70, 3) + 60*(math.sin(2*t))  # in cm


def calcW():
    return random.random()


def calcTheta():
    return random.randint(0, 360)

# A Canvas class for drawing a map and particles:
#     - it takes care of a proper scaling and coordinate transformation between
#      the map frame of reference (in cm) and the display (in pixels)


class Canvas:
    def __init__(self, map_size=210):
        self.map_size = map_size    # in cm
        self.canvas_size = 768         # in pixels
        self.margin = 0.05*map_size
        self.scale = self.canvas_size/(map_size+2*self.margin)

    def drawLine(self, line):
        x1 = self.__screenX(line[0])
        y1 = self.__screenY(line[1])
        x2 = self.__screenX(line[2])
        y2 = self.__screenY(line[3])
        print("drawLine:" + str((x1, y1, x2, y2)))

    def drawParticles(self, data):
        display = [(self.__screenX(d[0]), self.__screenY(d[1])) + d[2:]
                   for d in data]
        print("drawParticles:" + str(display))

    def __screenX(self, x):
        return (x + self.margin)*self.scale

    def __screenY(self, y):
        return (self.map_size + self.margin - y)*self.scale

# A Map class containing walls


class Map:
    def __init__(self):
        self.walls = []

    def add_wall(self, wall):
        self.walls.append(wall)

    def clear(self):
        self.walls = []

    def draw(self):
        for wall in self.walls:
            canvas.drawLine(wall)

# Simple Particles set


class Particles:
    def __init__(self):
        self.n = 100
        self.data = []

    def update(self):
        self.data = [(calcX(), calcY(), calcTheta(), calcW())
                     for i in range(self.n)]

    def draw(self):
        canvas.drawParticles(self.data)


canvas = Canvas()  # global canvas we are going to draw on

mymap = Map()
# Definitions of walls
# a: O to A
# b: A to B
# c: C to D
# d: D to E
# e: E to F
# f: F to G
# g: G to H
# h: H to O
mymap.add_wall((0, 0, 0, 168))        # a
mymap.add_wall((0, 168, 84, 168))     # b
mymap.add_wall((84, 126, 84, 210))    # c
mymap.add_wall((84, 210, 168, 210))   # d
mymap.add_wall((168, 210, 168, 84))   # e
mymap.add_wall((168, 84, 210, 84))    # f
mymap.add_wall((210, 84, 210, 0))     # g
mymap.add_wall((210, 0, 0, 0))        # h
mymap.draw()

PARTICLES = Particles()
PARTICLES.data = [(84, 30, 0, 1 / PARTICLES.n) for i in range(PARTICLES.n)]
# xCoords = [120, 130, 84, 84]
# yCoords = [30, 100, 100, 30]
xCoords = [[120, 140, 160], [130, 130, 130], [50]]
yCoords = [[30, 30, 30], [110, 130, 150], [90]]

try:
    # PARTICLES.draw()
    for i in range(len(xCoords)):
        for j in range(len(xCoords[0])):
            print("I'm at x: ", getXCoord(), ", y: ", getYCoord(), ", with angle: ", getAngleInDeg())
            print("Moving to... x:", xCoords[i][j], "y:", yCoords[i][j])
            goToCoord(xCoords[i][j], yCoords[i][j])
            print("")
            if (sweep(i)):
                break
    goToCoord(84, 30)
    print("Done.")
except KeyboardInterrupt:
    BP.reset_all()

