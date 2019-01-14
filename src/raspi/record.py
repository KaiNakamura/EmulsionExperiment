
#! /usr/bin/env python
from subprocess import call
call(['espeak "Welcome to the world of Robots" 2>/dev/null'], shell=True)

import os
import RPi.GPIO as GPIO
from picamera import PiCamera
from time import sleep
import datetime

GPIO.setmode(GPIO.BCM)
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
camera = PiCamera()

try:
    while True:
        if GPIO.input(27):
            print("Input detected")
            camera.start_preview(alpha = 128)
            camera.start_recording("/home/pi/Desktop/Repos/EmulsionExperiment/video/" + str(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-2]) + ".h264")
            sleep(45)
            camera.stop_recording()
            camera.stop_preview()  
        sleep(0.5)
finally:
    GPIO.cleanup()
