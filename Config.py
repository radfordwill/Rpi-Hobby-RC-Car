#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
Created on Sun Apr 09 09:44:33 2023
@author: Will Radford
This file serves as a configuration and variables file for the Rpi Zero RC Car app
"""

# Configuration variables
#

#Gamepad
buttonArm = 'OPTIONS'
buttonBeep = 'CIRCLE'
buttonExit = 'PS'
joystickSpeed = 'LEFT-Y'
joystickSteering = 'RIGHT-X'
pollInterval = 0.1
gamePadExit = False

#RC motor and steering
EscPin=8 #pin number on pca9685 for ESC
ServoPin=3 #pin number on pca9685 for steering servo
forwardSpeedValue = -.02
backwardSpeedValue = .22