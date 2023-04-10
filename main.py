#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
Created on Sun Apr 09 09:44:23 2023
@author: Will Radford
This file serves as the main file for the Rpi Zero RC Car app
"""
#Libraries
import curses
import os     #importing os library so as to communicate with the system
import time   #importing time library to make Rpi wait because its too impatient 
import board
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit
import adafruit_mpu6050
import Gamepad
from rpi_ws281x import PixelStrip, Color
from flask import Flask, render_template, Response
from processor.simple_streamer import SimpleStreamer as VideoCamera
# from processor.pedestrian_detector import PedestrianDetector as VideoCamera
# from processor.motion_detector import MotionDetector as VideoCamera
# from processor.qr_detector import QRDetector as VideoCamera
# from processor.face_detector import FaceDetector as VideoCamera
# from processor.person_detector import PersonDetector as VideoCamera
import threading
import Config as cfg

LED_COUNT = 8   # Number of LED pixels.
LED_PIN = 18   # GPIO pin connected to the pixels (18 uses PWM!).
# LED_PIN = 10        # GPIO pin connected to the pixels (10 uses SPI /dev/spidev0.0).
LED_FREQ_HZ = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA = 10          # DMA channel to use for generating signal (try 10)
LED_BRIGHTNESS = 255  # Set to 0 for darkest and 255 for brightest
LED_INVERT = False    # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53


#Gamepad setup and controller check
gamepadType = Gamepad.PS4
if not Gamepad.available():
    print('Please connect your controller...')
    while not Gamepad.available():
        time.sleep(1.0)
gamepad = gamepadType()

print('Gamepad connected')

#Setup for RC motor and steering
#pca9685 setup
i2c_bus = busio.I2C(SCL,SDA)
pca = PCA9685(i2c_bus)
pca.frequency = 60

#IMU setup
i2c_mpu = board.I2C()  # uses board.SCL and board.SDA
mpu = adafruit_mpu6050.MPU6050(i2c_mpu)

# max servo channels.
kit =ServoKit(channels=16)
kit.servo[3].angle = 93 #Set steering to center-ish
time.sleep(1)

print ("Done with RC Controls & Sensors")

#picamera and flask web page
video_camera = VideoCamera(flip=False)
app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

def gen(camera):
    while True:
        frame = camera.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(gen(video_camera),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

print ("Done with video feed")

# Create NeoPixel object with appropriate configuration.
strip = PixelStrip(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
# Intialize the library (must be called once before other functions).
#strip.begin()

def mpu6050():
    print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % (mpu.acceleration))
    print("Gyro X:%.2f, Y: %.2f, Z: %.2f rad/s" % (mpu.gyro))
    print("")
    time.sleep(1)

def colorWipe(strip, color, wait_ms=50):
    """Wipe color across display a pixel at a time."""
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, color)
        strip.show()
        time.sleep(wait_ms / 1000.0)

def theaterChase(strip, color, wait_ms=50, iterations=10):
    """Movie theater light style chaser animation."""
    for j in range(iterations):
        for q in range(3):
            for i in range(0, strip.numPixels(), 3):
                strip.setPixelColor(i + q, color)
            strip.show()
            time.sleep(wait_ms / 1000.0)
            for i in range(0, strip.numPixels(), 3):
                strip.setPixelColor(i + q, 0)

def right_side():
    kit.servo[cfg.ServoPin].angle = 180    
    return 'true'
def left_side():
    kit.servo[cfg.ServoPin].angle = 0
    return 'true'
    
def up_side(forwardSpeedValue): 
   stop_motor()
   kit.continuous_servo[cfg.EscPin].throttle = forwardSpeedValue 
   print("speed=")
   print(forwardSpeedValue)
   return 'true'
   
def down_side():
   stop_motor()
   kit.continuous_servo[cfg.EscPin].throttle = cfg.backwardSpeedValue
   print("speed=")
   print(cfg.backwardSpeedValue)
   return 'true'
   
def stop_motor():
   pca.channels[cfg.EscPin].duty_cycle = 0x0000
   return 'true'
   
def center_wheels():
   kit.servo[3].angle = 95
   return 'true'
   
def arm_esc():
   stop_motor()
   time.sleep(1)
   kit.continuous_servo[cfg.EscPin].throttle = .10
   time.sleep(1)
   kit.continuous_servo[cfg.EscPin].throttle = .30
   stop_motor()
   return 'true'

def runRcControl():
    # Start the background updating
    gamepad.startBackgroundUpdates()
    try:
        while gamepad.isConnected():
            if gamepad.beenPressed(cfg.buttonExit):
                print('EXIT')
                break
            if gamepad.beenPressed(cfg.buttonArm):
                arm_esc()
                print('ESC ARMED') 
                
            leftRight = float(gamepad.axis(cfg.joystickSteering))        
            if leftRight < -0.50:
                # Turning left
                left_side()
            elif leftRight > +0.50:
                # Turning right
                right_side()
                
            if leftRight == 0:
                center_wheels()         
                
            axisUpDownInverted = 1
            if axisUpDownInverted:
                upDown = float("{:.2f}".format(-gamepad.axis(cfg.joystickSpeed)))
            else:
                upDown = float("{:.2f}".format(gamepad.axis(cfg.joystickSpeed)))
                        
            if upDown >= 0:
                if upDown == 0:
                    #print("It is Zero!")
                    stop_motor()                
                else:
                    print("Positive number.")
                    print (upDown)                
                    if upDown > 0.60 and upDown < 0.90:
                        if gamepad.isPressed(cfg.buttonBeep):
                            cfg.forwardSpeedValue = -.10
                        else: 
                            cfg.forwardSpeedValue = -.09                   
                        up_side(cfg.forwardSpeedValue)
                        print("0.60 or more.")                
                    if upDown > 0.90:
                        if gamepad.isPressed(cfg.buttonBeep):
                            cfg.forwardSpeedValue = -.13
                        else:
                            cfg.forwardSpeedValue = -.12                    
                        up_side(cfg.forwardSpeedValue)
                        print("0.90 or more.")
                    else:
                        if gamepad.isPressed(cfg.buttonBeep):
                            cfg.forwardSpeedValue = -.10
                        else: 
                            cfg.forwardSpeedValue = -.09
                        
                        up_side(cfg.forwardSpeedValue)
                        print("0.60 or less.")
            else: 
                print("Negative number.")            
                if upDown <= -0.24:
                    down_side()
                    print("-24 or less.")    
                    #theaterChase(strip, Color(127, 0, 0), 15)  # Red theater chase                

            if gamepad.isPressed(cfg.buttonBeep):
                print('GO!')                
            
            #mpu6050()
            time.sleep(cfg.pollInterval)
            #colorWipe(strip, Color(0, 0, 0), 1)
    finally:
        gamepad.disconnect()
        
def runApp():
    app.run(debug=True, use_reloader=False, port=5000, host='0.0.0.0')
    
if __name__ == '__main__':
    try:
        print("start first thread")
        t1 = threading.Thread(target=runApp).start()
        print("start second thread")
        t2 = threading.Thread(target=runRcControl).start()
    except Exception as e:
        print("Unexpected error:" + str(e))
    
    
