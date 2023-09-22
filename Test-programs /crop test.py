import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
import keyboard

from picamera.array import PiRGBArray
from picamera import PiCamera 


camera = PiCamera()
camera.resolution = (800, 800)
camera.framerate = 30

rawCapture = PiRGBArray(camera, size=camera.resolution)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	image = frame.array
	W=255
	B=0
	
	#circle = cv2.HoughCircles(image,3,dp=1.5,minDist=1000,minRadius=100,maxRadius=1000)
	
	
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	thresh, binaryImg= cv2.threshold(gray,B,W,cv2.THRESH_OTSU)
	blur=cv2.medianBlur(gray,5)
	
	cv2.imshow("image", image)
	#cv2.imshow("gray", gray)
	#cv2.imshow("thresh",binaryImg)
	cv2.imshow("blur", blur)
	
	
	
	key = cv2.waitKey(1)
	rawCapture.truncate(0)
	if key == ord("q"):
		break

cv2.destroyAllWindows()
