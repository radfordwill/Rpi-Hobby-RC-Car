#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
Created on Sun Apr 09 09:44:33 2023
@author: Will Radford
This file serves as a configuration and variables file for the Rpi Zero RC Car app
"""

# Configuration

# LED strip configuration:
LED_COUNT = 8   # Number of LED pixels.
LED_PIN = 18   # GPIO pin connected to the pixels (18 uses PWM!).
# LED_PIN = 10        # GPIO pin connected to the pixels (10 uses SPI /dev/spidev0.0).
LED_FREQ_HZ = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA = 10          # DMA channel to use for generating signal (try 10)
LED_BRIGHTNESS = 255  # Set to 0 for darkest and 255 for brightest
LED_INVERT = False    # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53

#Gamepad
buttonArm = 'OPTIONS'
buttonBeep = 'CIRCLE'
buttonExit = 'PS'
joystickSpeed = 'LEFT-Y'
joystickSteering = 'RIGHT-X'
pollInterval = 0.1

#RC motor and steering
EscPin=8 #pin number on pca9685 for ESC
ServoPin=3 #pin number on pca9685 for steering servo
forwardSpeedValue = -.02
backwardSpeedValue = .22