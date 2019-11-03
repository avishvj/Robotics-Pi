##!/usr/bin/env python
#
# https://www.dexterindustries.com/BrickPi/
# https://github.com/DexterInd/BrickPi3
#
# Copyright (c) 2016 Dexter Industries
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
# For more information, see https://github.com/DexterInd/BrickPi3/blob/master/LICENSE.md
#
# This code is an example for running a motor to a target position set by the encoder of another motor.
#
# Hardware: Connect EV3 or NXT motors to the BrickPi3 motor ports B and C. Make sure that the BrickPi3 is running on a 9v power supply.
#
# Results:  When you run this program, motor C power will be controlled by the position of motor B. Manually rotate motor B, and motor C's power will change.

from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers
import sys
import math

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

def turnRight(deg):
#	radius = 6.1
#	circum = math.pi * 2 * 3.24
#	distance = radius * math.pi * (deg / 360) * 2
#	revolution = distance / circum * 360
	start_posi_d = BP.get_motor_encoder(BP.PORT_D)
	start_posi_a = BP.get_motor_encoder(BP.PORT_A)
	BP.set_motor_power(BP.PORT_A, 20)
	BP.set_motor_power(BP.PORT_D, -20)
	while (int((abs(BP.get_motor_encoder(BP.PORT_D) - start_posi_d) + abs(BP.get_motor_encoder(BP.PORT_A) - start_posi_a))) < deg):
		print("A status: ", BP.get_motor_status(BP.PORT_A), " D status: ", BP.get_motor_status(BP.PORT_D))

def moveForward(cm):
	radius = 3.24
	circum = math.pi * 2 * radius
	revolution = cm / circum * 360 * 0.997
	start_posi = BP.get_motor_encoder(BP.PORT_D)
	BP.set_motor_power(BP.PORT_A, float(sys.argv[2]))
	BP.set_motor_power(BP.PORT_D, float(sys.argv[3]))
	while (abs(BP.get_motor_encoder(BP.PORT_D) - start_posi) < revolution):
		print("A status: ", BP.get_motor_status(BP.PORT_A), " D status: ", BP.get_motor_status(BP.PORT_D))

def stop():
	BP.set_motor_power(BP.PORT_A, 0)
	BP.set_motor_power(BP.PORT_D, 0)
	time.sleep(1)

try:
    try:
        BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B)) # reset encoder B
    except IOError as error:
        print(error)

    for i in range(4):
        # The following BP.get_motor_encoder function returns the encoder value (what we want to use to control motor C's power).
        try:
            power = BP.get_motor_encoder(BP.PORT_D) / 10
            if power > 100:
                power = 100
            elif power < -100:
                power = -100
        except IOError as error:
            print(error)
            power = 0
        #BP.set_motor_power(BP.PORT_A, 80)
        #BP.set_motor_power(BP.PORT_D, 80)
        #time.sleep(0.8)  # delay for 0.02 seconds (20ms) to reduce the Raspberry Pi CPU load.
        moveForward(37)
        #moveForward(20)
        stop()
        #time.sleep(50)
        turnRight(int(sys.argv[1]))
        stop()

except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.
