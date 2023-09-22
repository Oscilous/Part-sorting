import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
import keyboard

from picamera.array import PiRGBArray
from picamera import PiCamera 
from pynput.keyboard import Key, Listener

servo_pin = 13
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin,GPIO.OUT)

pwm = GPIO.PWM(servo_pin,50) # 50 Hz (20 ms PWM period)
pwm.start(7) # start PWM by rotating to 90 degrees
time.sleep(0.5)
pwm.ChangeDutyCycle(0) # this prevents jitter



camera = PiCamera()
camera.resolution = (2000, 2000)
camera.framerate = 10

rawCapture = PiRGBArray(camera, size=camera.resolution)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	image = frame.array
	W=255
	B=0
	
	
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	thresh, binaryImg= cv2.threshold(gray,B,W,cv2.THRESH_OTSU)
	
	
	scale_precent =50
	width = int(binaryImg.shape[1]*scale_precent /100)
	height = int(binaryImg.shape[0]*scale_precent /100)
	dim = (width, height)
	resized = cv2.resize(binaryImg,dim,interpolation=cv2.INTER_AREA)
	resize = cv2.resize(image,dim,interpolation=cv2.INTER_AREA)
	
	
	
	cv2.imshow("image", resize)
	#cv2.imshow("gray", gray)
	#cv2.imshow("thresh",binaryImg)
	cv2.imshow("resized",resized)
	
	
	key = cv2.waitKey(1)
	rawCapture.truncate(0)
	if key == ord("q"):
		break

  
def show(key):
    
    if key == Key.tab:
        print("servo left")
        pwm.ChangeDutyCycle(2.0) # rotate to 0 degrees
        time.sleep(0.5)
        pwm.ChangeDutyCycle(7.0) # rotate to 90 degrees
        time.sleep(0.5)
        pwm.ChangeDutyCycle(0)
          
    if key != Key.tab:
        print("servo right")
        pwm.ChangeDutyCycle(12.0) # rotate to 0 degrees
        time.sleep(0.5)
        pwm.ChangeDutyCycle(7.0) # rotate to 90 degrees
        time.sleep(0.5)
        pwm.ChangeDutyCycle(0)


  
# Collect all event until released
with Listener(on_press = show) as listener:
    listener.join()
cv2.destroyAllWindows()


