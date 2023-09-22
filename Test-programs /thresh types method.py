import cv2
import numpy as np
import RPi.GPIO as GPIO
import time


from picamera.array import PiRGBArray
from picamera import PiCamera 


def nothing(x):
    pass
 
cv2.namedWindow("Trackbars")
 
cv2.createTrackbar("B", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("W", "Trackbars", 0, 255, nothing)


camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 40

rawCapture = PiRGBArray(camera, size=camera.resolution)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	image = frame.array
	W=255
	B=0
	
	
	B = cv2.getTrackbarPos("B", "Trackbars")
	W = cv2.getTrackbarPos("W", "Trackbars")
	
	
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	thresh, binaryImg= cv2.threshold(gray,B,W,cv2.THRESH_OTSU)
	mask_inv=cv2.bitwise_not(binaryImg)
	
	
	
	cv2.imshow("image", image)
	cv2.imshow("gray", gray)
	cv2.imshow("thresh",binaryImg)
	cv2.imshow("invthresh", mask_inv)
	
	
	key = cv2.waitKey(1)
	rawCapture.truncate(0)
	if key == ord("q"):
		break

cv2.destroyAllWindows()