import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
import keyboard

from picamera.array import PiRGBArray
from picamera import PiCamera 

servo_pin = 13
vib_pin = 12
enable_pin = 16
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin,GPIO.OUT)
GPIO.setup(enable_pin,GPIO.IN)
GPIO.setup(vib_pin,GPIO.OUT)
#pwmVib = GPIO.PWM(vib_pin, 31) # 50 Hz (20 ms PWM period)
#pwmVib.start(50)


pwm = GPIO.PWM(servo_pin,50) # 50 Hz (20 ms PWM period)
pwm.start(8.5) # start PWM by rotating to 90 degrees
time.sleep(0.5)
pwm.ChangeDutyCycle(0)



def meanPix(arrayAvg, diff,enabl,W_enable,otsu):
    if enabl == 1 and W_enable < 20:
        time.sleep(0.2)
        if 0.09 <= otsu:
            print("Fail")
            pwm.ChangeDutyCycle(6.8) # rotate to 0 degrees
            time.sleep(0.5)
            pwm.ChangeDutyCycle(12.0) # rotate to 90 degrees
            time.sleep(1)
            pwm.ChangeDutyCycle(8.0) # rotate to 90 degrees
            time.sleep(1)
            pwm.ChangeDutyCycle(0)
        elif 19 >= arrayAvg > 1.5 and diff<0.1:
            print("Fail")
            pwm.ChangeDutyCycle(6.8) # rotate to 0 degrees
            time.sleep(0.5)
            pwm.ChangeDutyCycle(12.0) # rotate to 90 degrees
            time.sleep(1)
            pwm.ChangeDutyCycle(8.0) # rotate to 90 degrees
            time.sleep(1)
            pwm.ChangeDutyCycle(0)
        elif 0 <= arrayAvg <= 1.5 and diff<0.1:
            print("Pass")
            pwm.ChangeDutyCycle(7) # rotate to 0 degrees
            time.sleep(0.5)
            pwm.ChangeDutyCycle(2.0) # rotate to 90 degrees
            time.sleep(1)
            pwm.ChangeDutyCycle(5.5) # rotate to 90 degrees
            time.sleep(1)
            pwm.ChangeDutyCycle(0)
        
    
def nothing(x):
    pass
 
cv2.namedWindow("Trackbars")
 
cv2.createTrackbar("B", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("W", "Trackbars", 0, 255, nothing)
W = cv2.getTrackbarPos("W", "Trackbars")

camera = PiCamera()
camera.resolution = (600, 600)
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
Csys=(300,315) #x,y coordinates 0,0 i venstre top
Dia=(200)
cv2.circle(circle, Csys,Dia,255,-1)
#cv2.imshow("Circle",circle)

arraySize =8
botArray = np.zeros((arraySize,),dtype=float)
topArray = np.zeros((arraySize,),dtype=float)
botArray[1] = 100
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	image = frame.array #rå pixel array
	
	
	B = 150 #led spænding 2,67V
	W = 255
	W2 =240 #tærskel for tom sensor
	#B = cv2.getTrackbarPos("B", "Trackbars") # justerbar tærskel
	#W2 = cv2.getTrackbarPos("W", "Trackbars")
	
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) #omdanner pixel array til grayscale fra HSV
	
	Wthresh, Wbinary= cv2.threshold(gray,W2,W,cv2.THRESH_BINARY) #kørere et binært tærskel maske over grayscale billede, for at se om der er en pakning i sensoren
	masked_data2 = cv2.bitwise_and(circle,Wbinary) #afgrænser outputtet til pakningens midte 
	
	
	thresh1, binaryImg= cv2.threshold(gray,B,W,cv2.THRESH_BINARY_INV) #kørere et binært tærskel maske over grayscale billede
	
	
	masked_data1 = cv2.bitwise_and(circle,binaryImg) #afgrænser outputtet til pakningens midte
	thresh2, OtsuImg= cv2.threshold(gray,B,W,cv2.THRESH_OTSU) #rafinere outputtet med OTSU algoritmen
	OTSUImg=cv2.bitwise_not(OtsuImg)
	masked_data3 = cv2.bitwise_and(circle,OTSUImg)
	
	overlay = cv2.addWeighted(gray,0.5, masked_data1, 0.7,0) #tilføjer resultatet som et overlay på grayscale billede
	
	image_area=cv2.circle(image, Csys,Dia,(0,67,180),1) #tegner en cyrcel på det originale billede for at vise det søgte område
	
	#visning af de forskellige skidt i processen
	#cv2.imshow("result Bin",masked_data1)
	cv2.imshow("image+area", image_area) 
	cv2.imshow("gray", overlay)
	#cv2.imshow("Otsu", OtsuImg)
	cv2.imshow("inv_otsu",masked_data3)
	#cv2.imshow("White", masked_data2)
	
	
	
	W_en = np.mean(masked_data2) #tæskel for hvor meget ren hvid der må være i billede
	p = np.mean(masked_data1) #udregning af hvor stor en endel af pakningen der er dårlig.
	Otsu = np.mean(masked_data3)
	#print(p)
	print(Otsu)
	
	botArray[0]= p #gemmer nuværende værdi et det rullende gennemsnit af de første X værdier 
	botArray = np.roll(botArray,1)
	topArray[0]=botArray[4] #gemmer sidste værdi fra det rullende gennemsnit af de første X værdier til et for de sidste X værdier
	topArray = np.roll(topArray,1)
	
	
	botSum = np.cumsum(botArray,dtype=float) #sumation af arrayet
	topSum = np.cumsum(topArray,dtype=float) #summation af arrayet
	#print(meanArray)
	#print(x)
	arrayAvgbot = botSum[arraySize-1]/arraySize #finder middelværdi af de første X værdier
	arrayAvgtop = topSum[arraySize-1]/arraySize #finder middelværdi af de sidste X værdier
	
	arrayDiff=abs(arrayAvgbot - arrayAvgtop) #finder diffecansen mellem de to gennemsnit
	
	en = GPIO.input(enable_pin) #hardware nemable pin
	
	#printning af de udregnet værdier
	#print(W_en)
	print(arrayAvgbot)
	#print(arrayAvgtop)
	print(arrayDiff)
	print("--------------")
	meanPix(arrayAvgbot,arrayDiff,en,W_en,Otsu) #servo og pass fail function
	
	
	key = cv2.waitKey(1)
	rawCapture.truncate(0)
	if key == ord("q"): #program shutdown
		break

cv2.destroyAllWindows()



