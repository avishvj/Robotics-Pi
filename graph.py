#!/usr/bin/env python

# Some suitable functions and data structures for drawing a map and particles

import time
import random
import math
import brickpi333 as brickpi3 # import the BrickPi3 drivers
import sys
import numpy as np

BP = brickpi3.BrickPi333() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
BP.set_sensor_type(BP.PORT_1, BP.SENSOR_TYPE.NXT_ULTRASONIC)

DISTANCE = 40
E_ERROR = 0.02
F_ERROR = 0.004
G_ERROR = 0.04
#E_ERROR = 0.04
#F_ERROR = 0.008
#G_ERROR = 0.08

ROBUSTNESS = 0
#ROBUSTNESS = 0.2

SONAR_OFFSET = 8.2

SCALING_FACTOR = 1
VARIANCE = 10

# x, y and theta are the robots position
# baseX and baseY is the new coord system
def getIntersection(x, y, theta, baseX, baseY, endX, endY):
    c = (baseY - y) - (baseX - x) * math.tan(theta)
    if (baseX == endX) :
        return (c > 0 and c < endY - baseY)
    else :
        hit = -c / math.tan(theta)
        return (hit > 0 and hit < endX - endY)

def getGroundTruthValue(x, y, theta, Ax, Ay, Bx, By):
    top = ((By - Ay) * (Ax - x)) - ((Bx - Ax) * (Ay - y))
    bot = ((By - Ay) * math.cos(theta)) - ((Bx - Ax) * math.sin(theta))
    return top / bot

# gives you closest wall to a particle's position
def getClosestWallToParticle(x, y, theta):
    closestWall = 500
    for i in mymap.walls:
        if getIntersection(x, y, theta, i[0], i[1], i[2], i[3]):
            distance = getGroundTruthValue(x, y, theta, i[0], i[1], i[2], i[3])
            if distance < closestWall:
                closestWall = distance
    return closestWall

def normalise(particles):
    sum = 0
    for p in particles:
        sum += p[3]
    for p in particles:
        p = (p[0], p[1], p[2], p[3] / sum)

def resample(particles):
    newParticles = []
    for p in particles:
        randomValue = random.gauss(0, 1)
        sampleFrom = p
        found = False
        for pp in particles:
            randomValue -= pp[3]
            if (not found and randomValue < 0):
                sampleFrom = pp
                found = True
                continue
        newParticles.append(sampleFrom)
    return newParticles

def calculateLikelihoodForAllParticles(particles, z):
    for p in particles:
        weight = calculateLikelihood(p[0], p[1], p[2], z)
        p = (p[0], p[1], p[2], weight)

def calculateLikelihood(x, y, theta, z):
    global VARIANCE
    m = getClosestWallToParticle(x, y, theta)
    error = z-m
    return math.e**(-(error)**2 / 2* VARIANCE) + ROBUSTNESS

def setPower(distance):
    distance = distance - stopping_distance
    if (distance > 100) :                                   # while the distance is above
        BP.set_motor_power(BP.PORT_A, 75)
        BP.set_motor_power(BP.PORT_D, 75)
    elif (distance < 15 and distance > 0):                  # while robot is near object, slow it down.
        BP.set_motor_power(BP.PORT_A, 10)
        BP.set_motor_power(BP.PORT_D, 10)
    else :                                                  # stops the robot when it's at stopping distance
        BP.set_motor_power(BP.PORT_A, distance * 0.75)
        BP.set_motor_power(BP.PORT_D, distance * 0.75)

def calculateStraightLine(D, particles):
    global E_ERROR, F_ERROR, SCALING_FACTOR
    for i in range(len(particles)):
        e = random.gauss(0, E_ERROR)
        f = random.gauss(0, F_ERROR)

        x = particles[i][0]
        y = particles[i][1]
        theta = particles[i][2]
        weight = particles[i][3]

        particles[i] = (x + (D + e) * math.cos(theta) * SCALING_FACTOR, y + (D + e) * math.sin(theta) * SCALING_FACTOR, theta + f, weight)
        tuple = (particles[i][0], particles[i][1], particles[i][2])

def calculateRotation(angle, particles):
    global G_ERROR
    for i in range(len(particles)):
        g = random.gauss(0, G_ERROR)

        x = particles[i][0]
        y = particles[i][1]
        theta = particles[i][2]
        weight = particles[i][3]

        particles[i] = (x, y, theta + (angle) + g, weight)
        tuple = (particles[i][0], particles[i][1], particles[i][2])

def turn(xCoord, yCoord, newXCoord, newYCoord, particles):
    newAngle = 0
    if (xCoord == newXCoord and yCoord < newYCoord):
        newAngle = math.pi / 2
    elif (yCoord == newYCoord and xCoord > newXCoord):
        newAngle = 3 * math.pi / 2
    else:
        newAngle = math.atan((newYCoord - yCoord) / (newXCoord - xCoord))

    currentAngle = getAngle(particles)

    if (xCoord <= newXCoord and yCoord <= newYCoord):
        newAngle = newAngle
    elif (xCoord > newXCoord and yCoord <= newYCoord):
        newAngle = math.pi + newAngle
    elif (xCoord > newXCoord and yCoord > newYCoord):
        newAngle = math.pi + newAngle
    elif (xCoord <= newXCoord and yCoord > newYCoord):        newAngle = 2 * math.pi + newAngle
    #print("Angle to turn: " , newAngle)
    turnClockwise(newAngle - currentAngle)
    calculateRotation(newAngle - currentAngle, particles)

def getXCoord(particles):
    sum = 0
    for i in range (len(particles)):
        sum += particles[i][0]
    avg = sum / len(particles)
    return avg

def getYCoord(particles):
    sum = 0
    for i in range (len(particles)):
        sum += particles[i][1]

    avg = sum / len(particles)
    return avg

def getAngle(particles):
    sum = 0
    for i in range (len(particles)):
        sum += particles[i][2]
    avg = sum / len(particles)
    return avg

def sensor360():
    angle = math.pi / 180 # angle to turn, currently 1 degree
    sonarReadings = []
    while (turn < 2 * math.pi):
        sonarSum = 0
        for i in range(3):
            sonarSum += BP.get_sensor(BP.PORT_1) + SONAR_OFFSET;
        sonarReading = sonarSum/3
        sonarReadings.append(sonarReading)
        turnClockwise(angle)
        angle += math.pi / 180    
    print(sonarReadings)

def turnClockwise(rad):
    print("Angle to turn: ", rad)
    if (rad < 0):
        rad += 2 * math.pi
    start_posi_d = BP.get_motor_encoder(BP.PORT_D)
    start_posi_a = BP.get_motor_encoder(BP.PORT_A)
    if (rad < math.pi):
        #print("1")
        BP.set_motor_power(BP.PORT_A, -20)
        BP.set_motor_power(BP.PORT_D, 20)
    else:
        #print("2")
        BP.set_motor_power(BP.PORT_A, 20)
        BP.set_motor_power(BP.PORT_D, -20)
        rad = 2 * math.pi - rad
    rotations = rad * 302 / (math.pi / 2)
    while (int((abs(BP.get_motor_encoder(BP.PORT_D) - start_posi_d) + abs(BP.get_motor_encoder(BP.PORT_A) - start_posi_a))) < rotations):
        a = 0
    stop()

def moveForward(cm, particles):
    circum = 21
    revolution = cm / circum * 360
    start_posi_a = BP.get_motor_encoder(BP.PORT_A)
    start_posi_d = BP.get_motor_encoder(BP.PORT_D)
    BP.set_motor_power(BP.PORT_A, 30)
    BP.set_motor_power(BP.PORT_D, 30)
    while ((abs(BP.get_motor_encoder(BP.PORT_D) - start_posi_d) + abs(BP.get_motor_encoder(BP.PORT_A) - start_posi_a)) / 2 < revolution):
        if (abs(BP.get_motor_encoder(BP.PORT_D)) % 8 == 0):
            BP.set_motor_power(BP.PORT_D, 28)
        elif (abs(BP.get_motor_encoder(BP.PORT_D)) % 4 == 0):
            BP.set_motor_power(BP.PORT_D, 30)
    stop()
    calculateStraightLine(cm, particles)

def goToCoord(newXCoord, newYCoord, particles):
    xCoord = (getXCoord(particles.data))
    yCoord = (getYCoord(particles.data))
    print(xCoord, "->", newXCoord, ", ", yCoord, "->", newYCoord)
    distance = math.sqrt((newXCoord - xCoord) ** 2 + (newYCoord - yCoord) ** 2)
    turn(xCoord, yCoord, newXCoord, newYCoord, particles.data)
    print("distance:", distance)
    while (distance > 20):
        moveForward(20, particles.data)
        resample2(particles)
        xCoord = getXCoord(particles.data)
        yCoord = getYCoord(particles.data)
        distance = math.sqrt((newXCoord - xCoord) ** 2 + (newYCoord - yCoord) ** 2)
        turn(xCoord, yCoord, newXCoord, newYCoord, particles.data)

    xCoord = getXCoord(particles.data)
    yCoord = getYCoord(particles.data)
    distance = math.sqrt((newXCoord - xCoord) ** 2 + (newYCoord - yCoord) ** 2)
    moveForward(distance, particles.data)
    resample2(particles)

def resample2(particles):
    particles.draw();
    oldParticles = particles.data
    sonarSum = 0
    for i in range(3):
        sonarSum += BP.get_sensor(BP.PORT_1) + SONAR_OFFSET;
    sonarReading = sonarSum/3
    calculateLikelihoodForAllParticles(oldParticles, sonarReading);
    normalise(oldParticles);
    particles.data = resample(oldParticles)

def stop():
    BP.set_motor_power(BP.PORT_A, 0)
    BP.set_motor_power(BP.PORT_D, 0)
    time.sleep(1)


# Functions to generate some dummy particles data:
def calcX():
    return random.gauss(80,3) + 70*(math.sin(t)); # in cm

def calcY():
    return random.gauss(70,3) + 60*(math.sin(2*t)); # in cm

def calcW():
    return random.random();

def calcTheta():
    return random.randint(0,360);

# A Canvas class for drawing a map and particles:
#     - it takes care of a proper scaling and coordinate transformation between
#      the map frame of reference (in cm) and the display (in pixels)
class Canvas:
    def __init__(self,map_size=210):
        self.map_size    = map_size;    # in cm;
        self.canvas_size = 768;         # in pixels;
        self.margin      = 0.05*map_size;
        self.scale       = self.canvas_size/(map_size+2*self.margin);

    def drawLine(self,line):
        x1 = self.__screenX(line[0]);
        y1 = self.__screenY(line[1]);
        x2 = self.__screenX(line[2]);
        y2 = self.__screenY(line[3]);
        print ("drawLine:" + str((x1,y1,x2,y2)))

    def drawParticles(self,data):
        #print("particles:", data)
        display = [(self.__screenX(d[0]),self.__screenY(d[1])) + d[2:] for d in data];
        print ("drawParticles:" + str(display))

    def __screenX(self,x):
        return (x + self.margin)*self.scale

    def __screenY(self,y):
        return (self.map_size + self.margin - y)*self.scale

# A Map class containing walls
class Map:
    def __init__(self):
        self.walls = [];

    def add_wall(self,wall):
        self.walls.append(wall);

    def clear(self):
        self.walls = [];

    def draw(self):
        for wall in self.walls:
            canvas.drawLine(wall);

# Simple Particles set
class Particles:
    def __init__(self):
        self.n = 100;
        self.data = [];

    def update(self):
        self.data = [(calcX(), calcY(), calcTheta(), calcW()) for i in range(self.n)];

    def draw(self):
        canvas.drawParticles(self.data);

canvas = Canvas();    # global canvas we are going to draw on

mymap = Map();
# Definitions of walls
# a: O to A
# b: A to B
# c: C to D
# d: D to E
# e: E to F
# f: F to G
# g: G to H
# h: H to O
mymap.add_wall((0,0,0,168));        # a
mymap.add_wall((0,168,84,168));     # b
mymap.add_wall((84,126,84,210));    # c
mymap.add_wall((84,210,168,210));   # d
mymap.add_wall((168,210,168,84));   # e
mymap.add_wall((168,84,210,84));    # f
mymap.add_wall((210,84,210,0));     # g
mymap.add_wall((210,0,0,0));        # h
mymap.draw();

particles = Particles();
particles.data = [(84, 30, 0, 1/particles.n) for i in range(particles.n)];
t = 0;
xCoords = [180, 180, 138, 138, 114, 114, 84, 84]
yCoords = [30, 54, 54, 168, 168, 84, 84, 30]

try:
    particles.draw()
    for i in range(len(xCoords)):
       print("x:", xCoords[i], "y:", yCoords[i])
       #goToCoord(xCoords[i], yCoords[i], particles)
       sensor360() 
        
except KeyboardInterrupt:
    BP.reset_all()





    #particles.update();

