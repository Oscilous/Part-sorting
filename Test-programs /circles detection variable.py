import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
import matplotlib.pyplot as plt
import math


from picamera.array import PiRGBArray
from picamera import PiCamera 



camera = PiCamera()
camera.resolution = (960, 960)
camera.framerate = 20
camera.brightness = 48 #48 til clen mask5
camera.contrast = 5 #1 giver bedst detection
#camera.image_effect='blur'
#camera.IMAGE_EFFECTS
camera.exposure_mode = 'backlight'
#camera.EXPOSURE_MODES
camera.awb_mode = 'fluorescent'


rawCapture = PiRGBArray(camera, size=camera.resolution)

def draw(detected_circles):
    if detected_circles is not None:
        detected_circles = np.uint16(np.around(detected_circles))
        
        for pt in detected_circles[0, :]:
            a, b, r = pt[0], pt[1], pt[2]
            
            # Draw the circumference of the circle.
            cv2.circle(image, (a, b), r, (0, 255, 0), 2)
  
            # Draw a small circle (of radius 1) to show the center.
            cv2.circle(image, (a, b), 1, (0, 50, 142), 3)
            
            
def nothing(x):
    pass
 
cv2.namedWindow("Trackbars")
 
#cv2.createTrackbar("minRadius", "Trackbars", 300, 450, nothing)
#cv2.createTrackbar("maxRadius", "Trackbars", 400, 500, nothing)
cv2.createTrackbar("param1", "Trackbars", 20, 250, nothing)
cv2.createTrackbar("param2", "Trackbars", 10, 100, nothing)


for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	image = frame.array #rå pixel array
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	gray_blurred = cv2.blur(gray, (3, 3))
	
	j=400
	i=440
	x=100
	y=10
	
	
	#j = cv2.getTrackbarPos("minRadius", "Trackbars") # justerbar tærskel
	#i = cv2.getTrackbarPos("maxRadius", "Trackbars")
	x = cv2.getTrackbarPos("param1", "Trackbars")
	y = cv2.getTrackbarPos("param2", "Trackbars")
	
	detected_circles = cv2.HoughCircles(gray_blurred,cv2.HOUGH_GRADIENT, 1, 200, param1 = x,param2 = y, minRadius = j, maxRadius = i)
	draw(detected_circles)
	
	
	
	#cv2.imshow("image+area", image) 
	cv2.imshow("Detected Circle", image)
	key = cv2.waitKey(1)
	rawCapture.truncate(0)
	if key == ord("q"): #program shutdown
		break

cv2.destroyAllWindows()







