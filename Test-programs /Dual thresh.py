import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
import keyboard

from picamera.array import PiRGBArray
from picamera import PiCamera 

servo_pin = 13
enable_pin = 16
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin,GPIO.OUT)
GPIO.setup(enable_pin,GPIO.IN)



pwm = GPIO.PWM(servo_pin,50) # 50 Hz (20 ms PWM period)
pwm.start(7) # start PWM by rotating to 90 degrees
time.sleep(0.5)
pwm.ChangeDutyCycle(0)



def meanPix(arrayAvg, diff,enabl,W_enable):
    if enabl == 1 and W_enable < 20:
        if 15 >= arrayAvg > 1.5 and diff<0.1:
            print("Fail")
            pwm.ChangeDutyCycle(3.5) # rotate to 0 degrees
            time.sleep(0.5)
            pwm.ChangeDutyCycle(7.0) # rotate to 90 degrees
            time.sleep(2.5)
            pwm.ChangeDutyCycle(0)
        elif 0 < arrayAvg <= 1.5 and diff<0.1:
            print("Pass")
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
W = cv2.getTrackbarPos("W", "Trackbars")

camera = PiCamera()
camera.resolution = (480, 480)
camera.framerate = 30
#camera.brightness = 25
#camera.contrast = 90 #40 ved B=180 giver bedst detection
#camera.image_effect='blur'
#camera.IMAGE_EFFECTS
#camera.exposure_mode = 'backlight'
#camera.EXPOSURE_MODES
#camera.awb_mode = 'flash'

rawCapture = PiRGBArray(camera, size=camera.resolution)

circle = np.zeros(camera.resolution, dtype="uint8")
Csys=(250,267)
Dia=(150)
cv2.circle(circle, Csys,Dia,255,-1)
#cv2.imshow("Circle",circle)

arraySize =12
botArray = np.zeros((arraySize,),dtype=float)
topArray = np.zeros((arraySize,),dtype=float)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	image = frame.array
	
	
	B = 115
	W = 255
	W2 =240
	B = cv2.getTrackbarPos("B", "Trackbars")
	#W2 = cv2.getTrackbarPos("W", "Trackbars")
	
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	
	Wthresh, Wbinary= cv2.threshold(gray,W2,W,cv2.THRESH_BINARY)
	masked_data2 = cv2.bitwise_and(circle,Wbinary)
	
	
	thresh1, binaryImg= cv2.threshold(gray,B,W,cv2.THRESH_BINARY_INV)
	#OTSUImg=cv2.bitwise_not(OTSU_norm)
	
	masked_data1 = cv2.bitwise_and(circle,binaryImg)
	thresh2, OtsuImg= cv2.threshold(masked_data1,B,W,cv2.THRESH_OTSU)
	
	overlay = cv2.addWeighted(gray,0.5, masked_data1, 0.7,0)
	
	image_area=cv2.circle(image, Csys,Dia,(255,4,23),1)
	#overlayComb = np,hstack((gr
	#cv2.imshow("result Bin",masked_data1)
	cv2.imshow("image+area", image_area)
	#cv2.imshow("gray", overlay)
	cv2.imshow("Otsu", OtsuImg)
	#cv2.imshow("White", masked_data2)
	
	W_en = np.mean(masked_data2)
	p = np.mean(OtsuImg)
	print(p)
	
	botArray[0]= p
	botArray = np.roll(botArray,1)
	topArray[0]=botArray[4]
	topArray = np.roll(topArray,1)
	
	
	botSum = np.cumsum(botArray,dtype=float)
	topSum = np.cumsum(topArray,dtype=float)
	#print(meanArray)
	#print(x)
	arrayAvgbot = botSum[arraySize-1]/arraySize
	arrayAvgtop = topSum[arraySize-1]/arraySize
	
	arrayDiff=abs(arrayAvgbot - arrayAvgtop)
	
	en = GPIO.input(enable_pin)
	time.sleep(0.1)
	#print(W_en)
	print(arrayAvgbot)
	print(arrayAvgtop)
	print(arrayDiff)
	print("--------------")
	#meanPix(arrayAvgbot,arrayDiff,en,W_en)
	
	
	key = cv2.waitKey(1)
	rawCapture.truncate(0)
	if key == ord("q"):
		break

cv2.destroyAllWindows()




