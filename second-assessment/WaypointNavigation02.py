
#!/usr/bin/env python
#
# https://www.dexterindustries.com/BrickPi/
# https://github.com/DexterInd/BrickPi3
#
# Copyright (c) 2016 Dexter Industries
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
# For more information, see https://github.com/DexterInd/BrickPi3/blob/master/LICENSE.md
#
# This code is an example for reading an NXT ultrasonic sensor connected to PORT_1 of the BrickPi3
#
# Hardware: Connect an NXT ultrasonic sensor to BrickPi3 Port 1.
#
# Results:  When you run this program, you should see the distance in CM.

from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers
import sys
import math
import numpy as np
import random

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

# Configure for an NXT ultrasonic sensor.
# BP.set_sensor_type configures the BrickPi3 for a specific sensor.
# BP.PORT_1 specifies that the sensor will be on sensor port 1.
# BP.SENSOR_TYPE.NXT_ULTRASONIC specifies that the sensor will be an NXT ultrasonic sensor.
BP.set_sensor_type(BP.PORT_1, BP.SENSOR_TYPE.NXT_ULTRASONIC)

stopping_distance = 30

DISTANCE = 40
E_ERROR = 0.01
F_ERROR = 0.002
G_ERROR = 0.01
NUMBER_OF_PARTICLES = 100
SCALING_FACTOR = 10
particles = np.full((NUMBER_OF_PARTICLES, 4), (100, 100, 0, 1 / NUMBER_OF_PARTICLES), dtype=float)


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

def printLine():
	global particles, SCALING_FACTOR, DISTANCE
	x = particles[0][0]
	y = particles[0][1]
	scaledDistance = DISTANCE * SCALING_FACTOR
	line1 = (x, y, x + scaledDistance, y)
	line2 = (x + scaledDistance, y, x + scaledDistance, y + scaledDistance)
	line3 = (x + scaledDistance, y + scaledDistance, x, y + scaledDistance)
	line4 = (x, y + scaledDistance, x, y)
	print("drawLine:" + str(line1))
	print("drawLine:" + str(line2))
	print("drawLine:" + str(line3))
	print("drawLine:" + str(line4))

def printParticles():
	global particles
	particlesTuples = ""
	for i in range(len(particles)):
		particlesTuples += str(tuple(particles[i]))
	print("drawParticles:" + particlesTuples)

def calculateStraightLine(D):
	global particles, E_ERROR, F_ERROR, SCALING_FACTOR
	for i in range(len(particles)):
		e = random.gauss(0, E_ERROR)
		f = random.gauss(0, F_ERROR)

		x = particles[i][0]
		y = particles[i][1]
		theta = particles[i][2]
		weight = particles[i][3]
#		print("weight: " + str(weight))

		particles[i] = (x + (D + e) * math.cos(theta) * SCALING_FACTOR, y + (D + e) * math.sin(theta) * SCALING_FACTOR, theta + f, weight)
		tuple = (particles[i][0], particles[i][1], particles[i][2])
#	printParticles()

def calculateRotation(angle):
	global particles, G_ERROR
	for i in range(len(particles)):
		g = random.gauss(0, G_ERROR)

		x = particles[i][0]
		y = particles[i][1]
		theta = particles[i][2]
		weight = particles[i][3]

		particles[i] = (x, y, theta + (angle) + g, weight)
		tuple = (particles[i][0], particles[i][1], particles[i][2])
#	printParticles()

def turn(xCoord, yCoord, newXCoord, newYCoord):
	newAngle = 0
	if (xCoord == newXCoord and yCoord < newYCoord):
		newAngle = math.pi / 2
	elif (yCoord == newYCoord and xCoord > newXCoord):
		newAngle = 3 * math.pi / 2
	else:
		newAngle = math.atan((newYCoord - yCoord) / (newXCoord - xCoord))
	currentAngle = getAngle()
#	print("Current angle: ", currentAngle)

	if (xCoord <= newXCoord and yCoord <= newYCoord):
		newAngle = newAngle
	elif (xCoord > newXCoord and yCoord <= newYCoord):
		newAngle = math.pi + newAngle
	elif (xCoord > newXCoord and yCoord > newYCoord):
		newAngle = math.pi + newAngle
	elif (xCoord <= newXCoord and yCoord > newYCoord):
		newAngle = 2 * math.pi + newAngle
#	print("Angle to turn: " , newAngle)
	turnClockwise(newAngle - currentAngle)
	calculateRotation(newAngle - currentAngle)



def getXCoord():
	global particles
	sum = 0
	for i in range (len(particles)):
		sum += particles[i][0]
	avg = sum / len(particles)
	return avg

def getYCoord():
	global particles
	sum = 0
	for i in range (len(particles)):
		sum += particles[i][1]
	avg = sum / len(particles)
	return avg

def getAngle():
	global particles
	sum = 0
	for i in range (len(particles)):
		sum += particles[i][2]
	avg = sum / len(particles)
	return avg


def turnClockwise(rad):
	rotations = rad * 300 / (math.pi / 2)
	start_posi_d = BP.get_motor_encoder(BP.PORT_D)
	start_posi_a = BP.get_motor_encoder(BP.PORT_A)
	BP.set_motor_power(BP.PORT_A, 20)
	BP.set_motor_power(BP.PORT_D, -20)
	while (int((abs(BP.get_motor_encoder(BP.PORT_D) - start_posi_d) + abs(BP.get_motor_encoder(BP.PORT_A) - start_posi_a))) < rotations):
		a = 0
	stop()
		#print("A status: ", BP.get_motor_status(BP.PORT_A), " D status: ", BP.get_motor_status(BP.PORT_D))

def moveForward(cm):
	circum = 21
	revolution = cm / circum * 360
	start_posi_a = BP.get_motor_encoder(BP.PORT_A)
	start_posi_d = BP.get_motor_encoder(BP.PORT_D)
	BP.set_motor_power(BP.PORT_A, 30)
	BP.set_motor_power(BP.PORT_D, 30)
	while ((abs(BP.get_motor_encoder(BP.PORT_D) - start_posi_d) + abs(BP.get_motor_encoder(BP.PORT_A) - start_posi_a)) / 2 < revolution):
		if (abs(BP.get_motor_encoder(BP.PORT_D)) % 6 == 0 and abs(BP.get_motor_encoder(BP.PORT_D)) % 9 == 0):
			BP.set_motor_power(BP.PORT_D, 29)
		elif (abs(BP.get_motor_encoder(BP.PORT_D)) % 6 == 0):
			BP.set_motor_power(BP.PORT_D, 30)
	stop()
#	print("A status: ", BP.get_motor_status(BP.PORT_A), " D status: ", BP.get_motor_status(BP.PORT_D))
	calculateStraightLine(cm)

def stop():
	BP.set_motor_power(BP.PORT_A, 0)
	BP.set_motor_power(BP.PORT_D, 0)
	time.sleep(1)

try:
        # read and display the sensor value
        # BP.get_sensor retrieves a sensor value.
        # BP.PORT_1 specifies that we are looking for the value of sensor port 1.
        # BP.get_sensor returns the sensor value (what we want to display).
#        print(str(particles[0][3]))
#        printParticles()
#        printLine()
#        step = DISTANCE / 4
        #for i in range(4):
            #value = BP.get_sensor(BP.PORT_1)
            #setPower(value)


#           moveForward(DISTANCE)
#           for i in range(4):
#              moveForward(DISTANCE)
#              stop()
            #moveForward(37)
            #stop()
#           turn(math.pi / 2)
#           print("X: ", getXCoord(), " Y: ", getYCoord(), " Angle: ", getAngle())
#           stop()

         while (1):
             newXCoord = int(input("Enter X Coord in cm: ")) * 10 + 100
             newYCoord = int(input("Enter Y Coord in cm: ")) * 10 + 100
             xCoord = getXCoord()
             yCoord = getYCoord()
#             print("X: ", xCoord, " X1: ", newXCoord, " Y ", yCoord, " Y1 ", newYCoord)
             distance = math.sqrt((newXCoord - xCoord) ** 2 + (newYCoord - yCoord) ** 2) / 10
             turn(xCoord, yCoord, newXCoord, newYCoord)
             moveForward(distance)
#             print ("xCoord after moving: ", getXCoord(), "yCoord: ", getYCoord(), "\n")


except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.
