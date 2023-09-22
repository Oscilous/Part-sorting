import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
import matplotlib.pyplot as plt
import math
import sys
import smtplib

from picamera.array import PiRGBArray
from picamera import PiCamera 

from picamera import PiCamera
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage
from datetime import date

SMTP_SERVER = 'smtp.gmail.com'
SMTP_PORT = 587
GMAIL_USERNAME = 'Aeeg.RasPi@gmail.com'
GMAIL_PASSWORD = 'bwmz vlam sqrh mfjr'

class Emailer: #funktion til at sende mail over Gmail server
    def sendmail(self, recipient, subject, content, image):

        #Create Headers
        emailData = MIMEMultipart()
        emailData['Subject'] = subject
        emailData['To'] = ",".join(recipient)
        emailData['From'] = GMAIL_USERNAME

        #Attach our text data
        emailData.attach(MIMEText(content))

        #Create our Image Data from the defined image
        imageData = MIMEImage(open(image, 'rb').read(), 'jpg')
        imageData.add_header('Content-Disposition', 'attachment; filename="image.jpg"')
        emailData.attach(imageData)

        #Connect to Gmail Server
        session = smtplib.SMTP(SMTP_SERVER, SMTP_PORT)
        session.ehlo()
        session.starttls()
        session.ehlo()

        #Login to Gmail
        session.login(GMAIL_USERNAME, GMAIL_PASSWORD)

        #Send Email & Exit
        session.sendmail(GMAIL_USERNAME, recipient, emailData.as_string())
        session.quit


GPIO.cleanup()
GPIO.setwarnings(False)
servo_pin = 13
vib_pin = 12
enable_pin = 20
ir_pin = 26
motor_pin =18
led_pin = 21
bowlsens = 24
pinch_pin = 23
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin,GPIO.OUT)
GPIO.setup(enable_pin,GPIO.IN,GPIO.PUD_DOWN)
GPIO.setup(vib_pin,GPIO.OUT)
GPIO.setup(ir_pin,GPIO.IN,GPIO.PUD_DOWN)
GPIO.setup(motor_pin, GPIO.OUT)
GPIO.setup(led_pin, GPIO.OUT)
GPIO.setup(pinch_pin, GPIO.OUT)
GPIO.setup(bowlsens,GPIO.IN,GPIO.PUD_DOWN)
pwmVib = GPIO.PWM(vib_pin, 30) 
pwmVib.start(0)

font = cv2.FONT_HERSHEY_PLAIN

pwm = GPIO.PWM(servo_pin,50) 
pwm.start(8.8) #init af servo motor
time.sleep(1)
pwm.ChangeDutyCycle(0)
counter=0
ticker=0
arrayNr = 1
GPIO.output(led_pin, 1)
startTime = time.time()
failCounter=0
PinchCD = time.time()
shutdownC = 0
adjust = 0

#histArray = np.zeros(200,dtype=float)
def histogram():
    #global histArray
    global Otsu
    global peripheral
    #global counter
    #global arrayNr
    with open("/home/pi/Desktop/pakning data/data " + str(date.today()) +".txt", "a+") as file_object:
        file_object.seek(0)
        file_object.write("\n")
        file_object.write(str(Otsu) + " " + str(peripheral))
        file_object.close()
    #histArray=np.append(histArray,Otsu)
    #histArray[counter]=Otsu
    #if(counter == 199):
        #datawrite = np.savetxt('/home/pi/Desktop/pakning data/data_array' + str(arrayNr) +'.txt',histArray,)
        #arrayNr=arrayNr+1
        #counter = 0
        #histArray = np.zeros(10,dtype=float)
   

def pinch():
    #print(pinch_pin)
    global PinchCD
    stopTime = time.time()
    elapsedTime=stopTime-PinchCD
    CD = 30 - (elapsedTime)

    #print(elapsedTime)
    #print(CD)
    if(CD<=0):
        print("deflate")
        GPIO.output(pinch_pin, 0)
        time.sleep(0.93)
        GPIO.output(pinch_pin, 1)
        PinchCD=time.time()
    
    

def sensor():
    global ticker
    global image_area
    global PinchCD
    #print(ticker)
    ir=GPIO.input(ir_pin)
    distB=GPIO.input(bowlsens)
    
    if(distB==0):
        GPIO.output(motor_pin, 0)
        pinch()
    else:
        GPIO.output(motor_pin, 1)
        PinchCD=time.time()
        
        
    #print(ir)
    #print(distB)
    ticker = (ticker+ir*2)-1
    
    if(ticker > 5):
        pwmVib.ChangeDutyCycle(50)
        image_area=cv2.rectangle(image_area,((15+78*ticker),945),(10,920),(0,255,0),-1)
    if(ticker >10):
        ticker=10
    if(ticker <= 0):
        ticker=0
    elif(ticker <=5):
        pwmVib.ChangeDutyCycle(0)
        image_area=cv2.rectangle(image_area,((15+78*ticker),945),(10,920),(0,0,255),-1)
        
def meanPix(arrayAvg, diff,enabl,W_enable,otsu,peripheral):
    global adjust 
    stopTime = time.time()
    adjust = adjust -1
    if (enabl == 1 and W_enable < 10 and adjust < 0)  :
        #print(W_enable)     
        #print(otsu,diff)
        #time.sleep(0.1)
        servoDelay = 0.17
        camDelay = 0.1
        
        
        global startTime
        global counter
        global failCounter
        global botArray
        global histArray
        global Csys
        global maskGain
        global shutdownC
        
        
        
        if (38>otsu>0.5  and diff<2 and maskGain < 0.85) :
            maskGain = maskGain+0.05
            otsu=50
            peripheral=0
            #GPIO.output(led_pin, 0)
            
            #GPIO.output(led_pin, 1)
            adjust=5
            #time.sleep(camDelay)
                   
            
        if (38>otsu>0.5  and diff<0.18 and maskGain > 0.84) or (peripheral>0.45 and diff<0.18 ):
            stopTime = time.time()
            elapsedTime=stopTime-startTime
            print("Fail  -  " + str(elapsedTime))
            startTime = time.time()
            cv2.putText(vis, "Threshold val: "+str(otsu) , (28,30),font,2,(255,255,255),2,cv2.LINE_AA)
            cv2.putText(vis, "Settling val: "+str(diff) , (32,58),font,2,(255,255,255),2,cv2.LINE_AA)
            cv2.putText(vis, "Peripheral val: "+str(peripheral) , (32,932),font,2,(255,255,255),2,cv2.LINE_AA)
            cv2.putText(vis, "Mask gain: "+str(maskGain) , (32,90),font,2,(255,255,255),2,cv2.LINE_AA)
            cv2.putText(vis, "Fail", (740,920),font,4,(0,0,255),2,cv2.LINE_AA)
            cv2.imwrite('/home/pi/Desktop/pakning image/fail/fail' + str(counter) + '.jpg',vis)
            print(counter, otsu, diff,peripheral)
            histogram()
               #Csys=(483,452)
            counter += 1
            GPIO.output(led_pin, 0)
            pwm.ChangeDutyCycle(6.8) # rotate to 0 degrees
            time.sleep(camDelay)
            GPIO.output(led_pin, 1)
            time.sleep(servoDelay+0.07)
                #botArray[1]=8
            pwm.ChangeDutyCycle(3)# rotate to 90 degrees
            
            
            time.sleep(servoDelay)            
            pwm.ChangeDutyCycle(5) # rotate to 90 degrees
            time.sleep(servoDelay)
            pwm.ChangeDutyCycle(0)
            time.sleep(camDelay)
            failCounter=failCounter+1
            maskGain = 0.70
            peripheral=0
           

            
        elif 0.01<=otsu<=0.5 and diff<0.08:
            stopTime = time.time()
            elapsedTime=stopTime-startTime
            print("Pass  -  " + str(elapsedTime))
            cv2.putText(vis, "Timer val: "+str(elapsedTime) , (28,942),font,2,(255,255,255),2,cv2.LINE_AA)
            startTime = time.time()
            #cv2.putText(vis, "Threshold val: "+str(otsu) , (28,30),font,2,(255,255,255),2,cv2.LINE_AA)
            #cv2.putText(vis, "Settling val: "+str(diff) , (32,58),font,2,(255,255,255),2,cv2.LINE_AA)
            #cv2.putText(vis, "Pass", (740,920),font,4,(0,255,0),2,cv2.LINE_AA)
            #cv2.imwrite('/home/pi/Desktop/pakning image/pass/pass' + str(counter) + '.jpg',vis)
            histogram()
            #Csys=(484,484)
            #print(counter, otsu, diff,peripheral)
            counter += 1
            GPIO.output(led_pin, 0)
            pwm.ChangeDutyCycle(6.8)
            time.sleep(camDelay)
            GPIO.output(led_pin, 1)# rotate to 0 degrees
            time.sleep(servoDelay+0.07)
            pwm.ChangeDutyCycle(11.2)# rotate to 90 degrees
            
            #botArray[1]=8
            
            time.sleep(servoDelay)
            pwm.ChangeDutyCycle(8.8) # rotate to 90 degrees
            time.sleep(servoDelay)
            pwm.ChangeDutyCycle(0)
            
            failCounter=0
            maskGain = 0.70
            peripheral=0
            shutdownC=0
            
        elif 0.01>=otsu and diff<0.08:
            shutdownC=3
            
        elif 40<otsu and W_enable < 20:
            
            GPIO.output(led_pin, 0)
            time.sleep(0.1)
            GPIO.output(led_pin, 1)
            time.sleep(0.3)
            failCounter=failCounter+1
            #time.sleep(servoDelay)
            if failCounter==10:
                pwm.ChangeDutyCycle(6.8)
                pwmVib.ChangeDutyCycle(0)
                GPIO.output(motor_pin, 1)
                print("Consecutive fail counter overflow")
                cv2.putText(vis, "Threshold val: "+str(otsu) , (28,30),font,2,(255,255,255),2,cv2.LINE_AA)
                cv2.putText(vis, "Settling val: "+str(diff) , (32,58),font,2,(255,255,255),2,cv2.LINE_AA)
                cv2.putText(vis, "Peripheral val: "+str(peripheral) , (32,932),font,2,(255,255,255),2,cv2.LINE_AA)
                cv2.putText(vis, "Fail", (740,920),font,4,(0,0,255),2,cv2.LINE_AA)
                cv2.imwrite('/home/pi/Desktop/pakning image/fail/fail' + str(counter) + '.jpg',vis)
                time.sleep(servoDelay+1)
                pwm.ChangeDutyCycle(3)
                time.sleep(servoDelay+1)
                pwm.ChangeDutyCycle(5)
                time.sleep(servoDelay)
                pwm.ChangeDutyCycle(0)
                failCounter=0
                shutdownC = shutdownC+1
                print(shutdownC)
                
        elif(shutdownC==3):
                cv2.imwrite('/home/pi/Desktop/pakning image/MaskinFejl.jpg',vis)
                sender = Emailer()
                image = '/home/pi/Desktop/pakning image/MaskinFejl.jpg'
                #sendTo = ['andreas.eeg97@gmail.com','bs@bmv.dk']
                sendTo = ['andreas.eeg97@gmail.com']
                emailSubject = "Fejl ved paknings maskinen"
                emailContent = "Tjek optisk-sensor for fastlåste pakninger, og genstart programmet når der er frit gennemløb"

                #Sends an email to the "sendTo" address with the specified "emailSubject" as the subject and "emailContent" as the email content.
                #sender.sendmail(sendTo, emailSubject, emailContent, image)
                sys.exit()
                
                sender = Emailer()
    
    elif((stopTime-startTime)>10):
        pwmVib.ChangeDutyCycle(50)
        time.sleep(0.4)
        pwmVib.ChangeDutyCycle(0)
        stopTime = time.time()
        
        
def nothing(x):
    pass
 
#cv2.namedWindow("Trackbars")
 
#cv2.createTrackbar("B2", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("Gain", "Trackbars", 0, 150, nothing)
#cv2.createTrackbar("Rad", "Trackbars", 300, 500, nothing)
#cv2.createTrackbar("W", "Trackbars", 0, 255, nothing)
#W = cv2.getTrackbarPos("W", "Trackbars")

camera = PiCamera()
camera.resolution = (960, 960)
camera.framerate = 30
camera.brightness = 47 #48 til clen mask5
camera.contrast = 0 #1 giver bedst detection
#camera.image_effect='blur'
#camera.IMAGE_EFFECTS
camera.exposure_mode = 'backlight'
#camera.EXPOSURE_MODES
camera.awb_mode = 'fluorescent'
maskGain = 0.7 #er alt hvidt = lavt gain, intet hvidt = for højt gain

rawCapture = PiRGBArray(camera, size=camera.resolution)

maskB = cv2.imread('mask clean11.jpg' , cv2.IMREAD_GRAYSCALE)
maskC = cv2.resize(maskB,camera.resolution)

circle = np.zeros(camera.resolution, dtype="uint8")
Csys=(480,468) #x,y coordinates 0,0 i venstre top
Dia=(401)
cv2.circle(circle, Csys,Dia,255,-1)
#cv2.imshow("Circle",circle)

circle2 = np.zeros(camera.resolution, dtype="uint8")
 #x,y coordinates 0,0 i venstre top
cv2.circle(circle2, Csys,478,255,35)
cv2.rectangle(circle2,(960,0),(10,40),(0,0,255),-1)

#circel2Neg=cv2.bitwise_not(circle2)
#cv2.imshow("Circle",circle2)

arraySize =8
botArray = np.zeros((arraySize,),dtype=float)
topArray = np.zeros((arraySize,),dtype=float)
botArray[1] = 100

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	image = frame.array #rå pixel array
	
	
	B = 215 #tærskel for pakning center
	W = 255
	W2 =250 #tærskel for tom sensor
	B2 =185 #tærskel for pakningens omrids
	#B2 = cv2.getTrackbarPos("B2", "Trackbars") # justerbar tærskel
	#maskGain = (cv2.getTrackbarPos("Gain", "Trackbars")/100)
	#W2 = cv2.getTrackbarPos("W", "Trackbars")
	
	gray1 = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) #omdanner pixel array til grayscale fra HSV
	grayINV = cv2.bitwise_not(gray1)
	maskINV  = cv2.bitwise_not(maskC)
	maskINVScale = cv2.multiply(maskINV,maskGain)
	gray2 = grayINV-maskINVScale
	gray = cv2.bitwise_not(gray2)
	#cv2.imshow("base", gray2)
	
	omrids = cv2.bitwise_and(circle2,gray2)
	testnavn, omridsT= cv2.threshold(omrids,B2,255,cv2.THRESH_BINARY)
	
	#cv2.imshow("base thresh", omridsT)
	#cv2.imshow("base", )
	#cv2.imshow("scalar", maskINVScale)
	
	Wthresh, Wbinary= cv2.threshold(gray1,W2,W,cv2.THRESH_BINARY) #kørere et binært tærskel maske over grayscale billede, for at se om der er en pakning i sensoren
	masked_data2 = cv2.bitwise_and(circle,Wbinary) #afgrænser outputtet til pakningens midte 
	
	
	#thresh1, binaryImg= cv2.threshold(gray,B,W,cv2.THRESH_OTSU) #kørere et binært tærskel maske over grayscale billede
	
	
	#masked_data1 = cv2.bitwise_and(circle,binaryImg) #afgrænser outputtet til pakningens midte
	thresh2, OtsuImg= cv2.threshold(gray,B,W,cv2.THRESH_BINARY)
	#cv2.imshow("base thresh", OtsuImg)
	#thresh2, OtsuImg2= cv2.threshold(gray,B,W,cv2.THRESH_OTSU)#rafinere outputtet med OTSU algoritmen
	OTSUImg=cv2.bitwise_not(OtsuImg)
	#OTSUImg2=cv2.bitwise_not(OtsuImg2)
	masked_data3 = cv2.bitwise_and(circle,OTSUImg)
	#masked_data32 = cv2.bitwise_and(circle,OTSUImg2)
	
	image_area=cv2.circle(image, Csys,Dia,(0,67,180),1) #tegner en cyrcel på det originale billede for at vise det søgte område
	
	
	overlay = cv2.addWeighted(gray1,0.5, (masked_data3+omridsT), 0.7,0)
	#overlay2 = cv2.addWeighted(gray1,0.5, masked_data32, 0.7,0)#tilføjer resultatet som et overlay på grayscale billede
	Overlay_area=cv2.circle(overlay,Csys,Dia,(10,220,30),1)
	Overlay_area=cv2.circle(overlay,Csys,458,(10,220,30),2)
	#Overlay_area2=cv2.circle(overlay2,Csys,Dia,(10,220,30),1)
	
	
	W_en = np.mean(masked_data2) #tæskel for hvor meget ren hvid der må være i billede
	#p = np.mean(masked_data1) #udregning af hvor stor en endel af pakningen der er dårlig.
	Otsu = np.mean(masked_data3)
	#print(W_en)
	peripheral = np.mean(omridsT)
	
	botArray[0]= (Otsu+peripheral) #gemmer nuværende værdi et det rullende gennemsnit af de første X værdier 
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
	#print(en)
	
	
	#printning af de udregnet værdier
	#print(en)
	#print(W_en)
	#print(">>><<<")
	#print(arrayAvgbot)
	#print(arrayAvgtop)
	#print(Otsu)
	#print(arrayDiff)
	#print("--------------")
	meanPix(arrayAvgbot,arrayDiff,en,W_en,Otsu,peripheral) #servo og pass fail function
	Overlay_area_rgb=cv2.cvtColor(Overlay_area,cv2.COLOR_GRAY2BGR)
	#Overlay_area_rgb2=cv2.cvtColor(Overlay_area2,cv2.COLOR_GRAY2BGR)
	vis =np.concatenate((Overlay_area_rgb,image_area),axis=1)
	
	sensor()
	
	cv2.putText(overlay, "Threshold val: "+str(Otsu) , (28,30),font,2,(255,255,255),2,cv2.LINE_AA)
	cv2.putText(overlay, "Settling val: "+str(arrayDiff) , (32,58),font,2,(255,255,255),2,cv2.LINE_AA)
	cv2.putText(overlay, "Peripheral val: "+str(peripheral) , (32,932),font,2,(255,255,255),2,cv2.LINE_AA)
	cv2.putText(overlay, "maskgain: "+str(maskGain) , (700,932),font,2,(255,255,255),2,cv2.LINE_AA)
	#visning af de forskellige skidt i processen
	#cv2.imshow("result Bin",vis)
	cv2.imshow("image+area", image_area) 
	cv2.imshow("result", Overlay_area)
	#cv2.imshow("Otsu", Overlay_area2)
	#cv2.imshow("inv_otsu",masked_data3)
	#cv2.imshow("White", masked_data2)
	
	
	key = cv2.waitKey(1)
	rawCapture.truncate(0)
	if key == ord("q"): #program shutdown
		break

cv2.destroyAllWindows()







