import os
import RPi.GPIO as GPIO
from picamera import PiCamera
from time import sleep

GPIO.setmode(GPIO.BCM)
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
camer = PiCamera()

try:
    while True:
        if GPIO.input(27):
            os.system("cd /home/pi/Desktop/Repos/EmulsionExperiment/video")
            camera.start_preview(alpha = 128)
            camera.start_recording("/home/pi/Desktop/Repos/EmulsionExperiment/video/video.h264")
            sleep(10)
            camera.stop_recording()
            camera.stop_preview()
            os.system("Mp4Box -add video.h264 video.mp4")
        sleep(0.5)
finally:
    GPIO.cleanup()