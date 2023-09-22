import cv2
import numpy as np 
from picamera.array import PiRGBArray
from picamera import PiCamera 
import RPi.GPIO as GPIO
import time

#led_pin = 12
#GPIO.setmode(GPIO.BCM)
#GPIO.setup(led_pin,GPIO.OUT)



#GPIO.output(led_pin,GPIO.HIGH)



camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 20
camera.contrast = 55

rawCapture = PiRGBArray(camera, size=(640, 480))

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	image = frame.array
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


	#lowerLimit = np.array([3,100,100])
	#upperLimit = np.array([8,250,250])

	#mask = cv2.inRange(hsv, lowerLimit, upperLimit)

	#result = cv2.bitwise_and(image	, image	, mask=mask)

	cv2.imshow("frame", image)
	#cv2.imshow("mask", mask)
	#cv2.imshow("result", result)

	key = cv2.waitKey(1)
	rawCapture.truncate(0)
	if key == ord("q"):
		break

cv2.destroyAllWindows()