import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
import matplotlib.pyplot as plt


from picamera.array import PiRGBArray
from picamera import PiCamera 

GPIO.cleanup()
GPIO.setwarnings(False)
servo_pin = 13
vib_pin = 12
enable_pin = 20
ir_pin = 26
motor_pin =18
led_pin = 21
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin,GPIO.OUT)
GPIO.setup(enable_pin,GPIO.IN)
GPIO.setup(vib_pin,GPIO.OUT)
GPIO.setup(ir_pin,GPIO.IN,GPIO.PUD_DOWN)
GPIO.setup(motor_pin, GPIO.OUT)
GPIO.setup(led_pin, GPIO.OUT)
pwmVib = GPIO.PWM(vib_pin, 30) # 50 Hz (20 ms PWM period)
pwmVib.start(50)

font = cv2.FONT_HERSHEY_PLAIN

pwm = GPIO.PWM(servo_pin,50) # 50 Hz (20 ms PWM period)
pwm.start(8.0) # start PWM by rotating to 90 degrees
time.sleep(1)
pwm.ChangeDutyCycle(0)
counter=0
ticker=0
arrayNr = 1
GPIO.output(led_pin, 1)

histArray = np.zeros(10,dtype=float)
def histogram():
    global histArray
    global Otsu
    global counter
    global arrayNr
    #histArray=np.append(histArray,Otsu)
    histArray[counter]=Otsu
    if(counter == 9):
        datawrite = np.savetxt('Otsu' + str(arrayNr) +'.txt',histArray,)
        arrayNr=arrayNr+1
        counter = 0
        #histArray = np.zeros(10,dtype=float)
   

    

def sensor():
    global ticker
    print(ticker)
    ir=GPIO.input(ir_pin)
    #print(ir)
    ticker = (ticker+ir*2)-1 
    if(ticker > 6):
        pwmVib.ChangeDutyCycle(50)
        GPIO.output(motor_pin, 0)
    if(ticker >15):
        ticker=15
    if(ticker <= 0):
        ticker=0
    elif(ticker <=6):
        pwmVib.ChangeDutyCycle(0)
        GPIO.output(motor_pin, 1)
        

def meanPix(arrayAvg, diff,enabl,W_enable,otsu):
    if enabl == 1 and W_enable < 20  :
        #print("enabled")     
        #print(otsu,diff)
        #time.sleep(0.01)
        servoDelay = 0.19
        camDelay = 0.4
        global counter
        global botArray
        global histArray
        if 38>otsu>0.8  and diff<0.1:
            print("Fail")
            cv2.putText(vis, "Threshold val: "+str(otsu) , (28,30),font,2,(255,255,255),2,cv2.LINE_AA)
            cv2.putText(vis, "Settling val: "+str(diff) , (32,58),font,2,(255,255,255),2,cv2.LINE_AA)
            cv2.putText(vis, "Fail", (740,900),font,4,(0,0,255),2,cv2.LINE_AA)
            cv2.imwrite('/home/pi/Desktop/pakning image/fail/fail' + str(counter) + '.jpg',vis)
            print(counter, otsu, diff)
            histogram()
            counter += 1
            GPIO.output(led_pin, 0)
            pwm.ChangeDutyCycle(6.8) # rotate to 0 degrees
            time.sleep(servoDelay)
            botArray[1]=8
            
            pwm.ChangeDutyCycle(10.90)# rotate to 90 degrees
            GPIO.output(led_pin, 1)
            time.sleep(servoDelay)            
            pwm.ChangeDutyCycle(8.0) # rotate to 90 degrees
            time.sleep(servoDelay)
            pwm.ChangeDutyCycle(0)
            time.sleep(camDelay)

            
        elif 0<=otsu<=0.8 and diff<0.01:
            print("Pass")
            cv2.putText(vis, "Threshold val: "+str(otsu) , (28,30),font,2,(255,255,255),2,cv2.LINE_AA)
            cv2.putText(vis, "Settling val: "+str(diff) , (32,58),font,2,(255,255,255),2,cv2.LINE_AA)
            cv2.putText(vis, "Pass", (740,900),font,4,(0,255,0),2,cv2.LINE_AA)
            cv2.imwrite('/home/pi/Desktop/pakning image/pass/pass' + str(counter) + '.jpg',vis)
            histogram()
            print(counter, otsu, diff)
            counter += 1
            GPIO.output(led_pin, 0)
            pwm.ChangeDutyCycle(6.8) # rotate to 0 degrees
            time.sleep(servoDelay)
            pwm.ChangeDutyCycle(3.0)# rotate to 90 degrees
            GPIO.output(led_pin, 1)
            botArray[1]=8
            
            time.sleep(servoDelay)
            pwm.ChangeDutyCycle(5.6) # rotate to 90 degrees
            time.sleep(servoDelay)
            pwm.ChangeDutyCycle(0)
            time.sleep(camDelay)
            
        elif 40<otsu and diff<18 and W_enable < 20:
            GPIO.output(led_pin, 0)
            time.sleep(0.1)
            GPIO.output(led_pin, 1)
            time.sleep(0.8)
    
    
def nothing(x):
    pass
 
cv2.namedWindow("Trackbars")
 
cv2.createTrackbar("B", "Trackbars", 0, 255, nothing)
#cv2.createTrackbar("Gain", "Trackbars", 0, 100, nothing)
#cv2.createTrackbar("W", "Trackbars", 0, 255, nothing)
#W = cv2.getTrackbarPos("W", "Trackbars")

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
maskGain = 0.70 #er alt hvidt = lavt gain, intet hvidt = for højt gain

rawCapture = PiRGBArray(camera, size=camera.resolution)

maskB = cv2.imread('mask clean9.jpg' , cv2.IMREAD_GRAYSCALE)
maskC = cv2.resize(maskB,camera.resolution)

circle = np.zeros(camera.resolution, dtype="uint8")
masked_data3 = np.zeros(camera.resolution, dtype="uint8")

arraySize =10
botArray = np.zeros((arraySize,),dtype=float)
topArray = np.zeros((arraySize,),dtype=float)
botArray[1] = 100

def draw(detected_circles):
    circle = np.zeros(camera.resolution, dtype="uint8")
    if detected_circles is not None:
        detected_circles = np.uint16(np.around(detected_circles))
        
        for pt in detected_circles[0, :]:
            a, b, r = pt[0], pt[1], pt[2]
            cv2.putText(image, "radius: "+str(r), (50,900),cv2.FONT_HERSHEY_PLAIN,2,(255,255,0),2,cv2.LINE_AA)
            cv2.putText(image, "Csys:( "+str(a)+";"+str(a)+")", (650,900),cv2.FONT_HERSHEY_PLAIN,2,(155,0,120),2,cv2.LINE_AA)
            
            # Draw the circumference of the circle.
            cv2.circle(image, (a, b), r, (0, 255, 0), 1)
            # Draw a small circle (of radius 1) to show the center.
            cv2.circle(image, (a, b), 1, (0, 50, 142), 3)
            
            Csys=(a,b) #x,y coordinates 0,0 i venstre top
            Dia=(r-14)
            cv2.circle(circle, Csys,Dia,255,-1)
            B = cv2.getTrackbarPos("B", "Trackbars") # justerbar tærskel
            thresh2, OtsuImg= cv2.threshold(gray1,B,255,cv2.THRESH_BINARY)
            
            #thresh2, OtsuImg2= cv2.threshold(gray,B,W,cv2.THRESH_OTSU)#rafinere outputtet med OTSU algoritmen
            OTSUImg=cv2.bitwise_not(OtsuImg)
            #OTSUImg2=cv2.bitwise_not(OtsuImg2)
            global masked_data3
            masked_data3 = cv2.bitwise_and(circle,OTSUImg)
            #circle=np.fill(0)
            
            

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	image = frame.array #rå pixel array
	
	
	B = 215 #led spænding 2,67V
	W = 255
	W2 =250 #tærskel for tom sensor
	#B = cv2.getTrackbarPos("B", "Trackbars") # justerbar tærskel
	#maskGain = (cv2.getTrackbarPos("Gain", "Trackbars")/100)
	#W2 = cv2.getTrackbarPos("W", "Trackbars")
	
	
	
	gray1 = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) #omdanner pixel array til grayscale fra HSV
	grayINV = cv2.bitwise_not(gray1)
	maskINV  = cv2.bitwise_not(maskC)
	maskINVScale = cv2.multiply(maskINV,maskGain)
	gray2 = grayINV-maskINVScale
	gray = cv2.bitwise_not(gray2)
	
	
	detected_circles = cv2.HoughCircles(gray1,cv2.HOUGH_GRADIENT, 1, 200, param1 = 100,param2 = 10, minRadius = 426, maxRadius = 438)
	draw(detected_circles)
	
	#cv2.imshow("base", maskINV)
	#cv2.imshow("scalar", maskINVScale)
	
	Wthresh, Wbinary= cv2.threshold(gray1,W2,W,cv2.THRESH_BINARY) #kørere et binært tærskel maske over grayscale billede, for at se om der er en pakning i sensoren
	masked_data2 = cv2.bitwise_and(circle,Wbinary) #afgrænser outputtet til pakningens midte 
	
	
	#thresh1, binaryImg= cv2.threshold(gray,B,W,cv2.THRESH_OTSU) #kørere et binært tærskel maske over grayscale billede
	
	
	#masked_data1 = cv2.bitwise_and(circle,binaryImg) #afgrænser outputtet til pakningens midte
	#masked_data32 = cv2.bitwise_and(circle,OTSUImg2)
	
	#image_area=cv2.circle(image, Csys,Dia,(0,67,180),1) #tegner en cyrcel på det originale billede for at vise det søgte område
	
	overlay = cv2.addWeighted(gray1,0.5, masked_data3, 0.7,0)
	#overlay2 = cv2.addWeighted(gray1,0.5, masked_data32, 0.7,0)#tilføjer resultatet som et overlay på grayscale billede
	#Overlay_area=cv2.circle(overlay,Csys,Dia,(10,220,30),1)
	#Overlay_area2=cv2.circle(overlay2,Csys,Dia,(10,220,30),1)
	
	
	W_en = np.mean(masked_data2) #tæskel for hvor meget ren hvid der må være i billede
	#p = np.mean(masked_data1) #udregning af hvor stor en endel af pakningen der er dårlig.
	Otsu = np.mean(masked_data3)
	#print(Otsu)
	
	
	botArray[0]= Otsu #gemmer nuværende værdi et det rullende gennemsnit af de første X værdier 
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
	sensor()
	
	
	#printning af de udregnet værdier
	#print(en)
	#print(W_en)
	#print(">>><<<")
	#print(arrayAvgbot)
	#print(arrayAvgtop)
	#print(Otsu)
	#print(arrayDiff)
	#print("--------------")
	meanPix(arrayAvgbot,arrayDiff,en,W_en,Otsu) #servo og pass fail function
	Overlay_area_rgb=cv2.cvtColor(gray1,cv2.COLOR_GRAY2BGR)
	#Overlay_area_rgb2=cv2.cvtColor(Overlay_area2,cv2.COLOR_GRAY2BGR)
	vis =np.concatenate((Overlay_area_rgb,image),axis=1)
	
	cv2.putText(overlay, "Threshold val: "+str(Otsu) , (28,30),font,2,(255,255,255),2,cv2.LINE_AA)
	cv2.putText(overlay, "Settling val: "+str(arrayDiff) , (32,58),font,2,(255,255,255),2,cv2.LINE_AA)        
	#visning af de forskellige skidt i processen
	#cv2.imshow("result Bin",vis)
	cv2.imshow("image+area", image) 
	#cv2.imshow("result", overlay)
	#cv2.imshow("Otsu", Overlay_area2)
	#cv2.imshow("inv_otsu",masked_data3)
	#cv2.imshow("White", masked_data2)
	
	
	key = cv2.waitKey(1)
	rawCapture.truncate(0)
	if key == ord("q"): #program shutdown
		break

cv2.destroyAllWindows()





