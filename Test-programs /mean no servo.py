import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
import keyboard

from picamera.array import PiRGBArray
from picamera import PiCamera 


#def nothing(x):
    #pass
 
#cv2.namedWindow("Trackbars")
 
#cv2.createTrackbar("B", "Trackbars", 0, 255, nothing)
#cv2.createTrackbar("W", "Trackbars", 0, 255, nothing)


camera = PiCamera()
camera.resolution = (640, 640)
camera.framerate = 30

rawCapture = PiRGBArray(camera, size=camera.resolution)

circle = np.zeros((640,640), dtype="uint8")
cv2.circle(circle, (320,320),280,255,-1)
#cv2.imshow("Circle",circle)


for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	image = frame.array
	
	
	#B = cv2.getTrackbarPos("B", "Trackbars")
	#W = cv2.getTrackbarPos("W", "Trackbars")
	W=255
	B=137
	
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	thresh, binaryImg= cv2.threshold(gray,B,W,cv2.THRESH_BINARY_INV)
	#mask = cv2.imread('/home/pi/Pictures/mask2.png')
	masked_data = cv2.bitwise_and(circle,binaryImg)
	
	
	cv2.imshow("result",masked_data)
	cv2.imshow("image", image)
	#cv2.imshow("gray", gray)
	cv2.imshow("thresh",binaryImg)
	
	
	p = np.mean(masked_data)
	print(p)
	#time.sleep(4)
	#meanPix(masked_data)
	#time.sleep(4)
	
	key = cv2.waitKey(1)
	rawCapture.truncate(0)
	if key == ord("q"):
		break

cv2.destroyAllWindows()

