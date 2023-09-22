import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
import keyboard

from picamera.array import PiRGBArray
from picamera import PiCamera 

servo_pin = 13
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin,GPIO.OUT)

pwm = GPIO.PWM(servo_pin,50) # 50 Hz (20 ms PWM period)
pwm.start(7) # start PWM by rotating to 90 degrees
time.sleep(0.5)
pwm.ChangeDutyCycle(0)



def meanPix(x, diff):
    if 15 >= x > 0.6 and diff<0.2:
        print("servo left")
        pwm.ChangeDutyCycle(3.5) # rotate to 0 degrees
        time.sleep(0.5)
        pwm.ChangeDutyCycle(7.0) # rotate to 90 degrees
        time.sleep(2.5)
        pwm.ChangeDutyCycle(0)
    elif 0 < x <= 0.6 and diff<0.2:
        print("servor right")
        pwm.ChangeDutyCycle(10.5) # rotate to 0 degrees
        time.sleep(0.5)
        pwm.ChangeDutyCycle(7.0) # rotate to 90 degrees
        time.sleep(2.5)
        pwm.ChangeDutyCycle(0)
      

def nothing(x):
    pass
 
cv2.namedWindow("Trackbars")
 
cv2.createTrackbar("B", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("W", "Trackbars", 0, 255, nothing)


camera = PiCamera()
camera.resolution = (640, 640)
camera.framerate = 10
camera.contrast = 100 #40 ved B=180 giver bedst detection
#camera.image_effect='blur'
#camera.exposure_mode = 'backlight'
camera.AWB_MODES

rawCapture = PiRGBArray(camera, size=camera.resolution)

circle = np.zeros(camera.resolution, dtype="uint8")
cv2.circle(circle, (320,320),240,255,-1)
cv2.imshow("Circle",circle)

arraySize =10
meanArray = np.zeros((arraySize,),dtype=float)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	image = frame.array
	B = 137
	W = 255
	
	B = cv2.getTrackbarPos("B", "Trackbars")
	#W = cv2.getTrackbarPos("W", "Trackbars")
	
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	thresh1, OTSU_norm= cv2.threshold(gray,B,W,cv2.THRESH_OTSU)
	OTSUImg=cv2.bitwise_not(OTSU_norm)
	thresh2, binaryImg= cv2.threshold(gray,B,W,cv2.THRESH_BINARY_INV)
	thresh3, TOZImg= cv2.threshold(gray,B,W,cv2.THRESH_TOZERO_INV)
	#mask = cv2.imread('/home/pi/Pictures/mask2.png')
	masked_data1 = cv2.bitwise_and(circle,OTSUImg)
	masked_data2 = cv2.bitwise_and(circle,binaryImg)
	masked_data3 = cv2.bitwise_and(circle,TOZImg)
	
	p = np.mean(masked_data1)
	#print(p)
	#time.sleep(2)
	
	cv2.imshow("result OTSU",masked_data1 )
	cv2.imshow("result BINARY",masked_data2)
	cv2.imshow("result TOZ",masked_data3)
	cv2.imshow("image", image)
	#cv2.imshow("gray", gray)
	#cv2.imshow("thresh",binaryImg)
	
	
	meanArray[0]= np.mean(masked_data3)
	meanArray = np.roll(meanArray,1)
	x = np.cumsum(meanArray,dtype=float)
	#print(meanArray)
	#print(x)
	y= x[arraySize-1]/arraySize
	arrayDiff=abs(p - y)
	
	print(y)
	print(arrayDiff)
	print("--------------")
	#meanPix(y,arrayDiff)
	#time.sleep(0.2)
	
	
	
	key = cv2.waitKey(1)
	rawCapture.truncate(0)
	if key == ord("q"):
		break

cv2.destroyAllWindows()

