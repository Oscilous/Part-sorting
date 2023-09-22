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
            cv2.putText(image, "radius: "+str(r), (50,900),cv2.FONT_HERSHEY_PLAIN,2,(255,255,0),2,cv2.LINE_AA)
            
            cv2.circle(image, (a, b), r, (0, 255, 0), 2)
            # Draw a small circle (of radius 1) to show the center.
            cv2.circle(image, (a, b), 1, (0, 50, 142), 3)
            
            circle = np.zeros(camera.resolution, dtype="uint8")
            Csys=(a,b) #x,y coordinates 0,0 i venstre top
            Dia=(r-5)
            cv2.circle(circle, Csys,Dia,255,-1)
            cv2.imshow("Circle",circle)
              
            thresh2, OtsuImg= cv2.threshold(gray_blurred,150,255,cv2.THRESH_OTSU)
            masked_data3 = cv2.bitwise_and(circle,OtsuImg)
            #cv2.imshow("thresh2",masked_data3)
   
            overlay = cv2.addWeighted(gray_blurred,0.5, masked_data3, 0.7,0)
            cv2.imshow("overlay",overlay)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	image = frame.array #rå pixel array
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	gray_blurred = cv2.blur(gray, (3, 3))
	
	
	detected_circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT, 1, 200, param1 = 100,param2 = 10, minRadius = 426, maxRadius = 438)
	draw(detected_circles)
	
	
	cv2.imshow("Detected Circle", image)
	key = cv2.waitKey(1)
	rawCapture.truncate(0)
	if key == ord("q"): #program shutdown
		break

cv2.destroyAllWindows()






