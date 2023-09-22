import cv2
import numpy as np 
from picamera.array import PiRGBArray
from picamera import PiCamera 
import RPi.GPIO as GPIO
import time

camera = PiCamera()
camera.resolution = (640, 640)
camera.framerate = 20

rawCapture = PiRGBArray(camera, size=(640, 640))

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	image = frame.array

	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	thresh, binaryImg= cv2.threshold(gray,0,255,cv2.THRESH_OTSU)


	
	cv2.imshow("image", image)
	cv2.imshow("gray", gray)
	cv2.imshow("thresh",binaryImg)

	
	key = cv2.waitKey(1)
	rawCapture.truncate(0)
	if key == ord("q"):
		break

cv2.destroyAllWindows()