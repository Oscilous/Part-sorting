import cv2
import numpy as np
import RPi.GPIO as GPIO
import time


from picamera.array import PiRGBArray
from picamera import PiCamera 


GPIO.setwarnings(False)
servo_pin = 13
vib_pin = 12
enable_pin = 20
ir_pin = 23
motor_pin =18
led_pin = 21
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin,GPIO.OUT)
GPIO.setup(enable_pin,GPIO.IN)
GPIO.setup(vib_pin,GPIO.OUT)
GPIO.setup(ir_pin,GPIO.IN)
GPIO.setup(motor_pin, GPIO.OUT)
GPIO.setup(led_pin, GPIO.OUT)

###############################################
#image capturing

camera = PiCamera() # start picamera

B=137
W=255


# camera presets
camera.resolution = (1200,1200)
#camera.brightness = 25
camera.contrast = 10 #40 ved B=180 giver bedst detection
#camera.image_effect='blur'
#camera.IMAGE_EFFECTS
camera.exposure_mode = 'backlight'
#camera.EXPOSURE_MODES
camera.awb_mode = 'flash'


camera.start_preview()
time.sleep(2)
GPIO.output(led_pin, 1)
time.sleep(1)
camera.capture('/home/pi/Desktop/test-img.jpg')
GPIO.output(led_pin, 0)
camera.stop_preview()

##################################################
#image prosessing 

circle = np.zeros(camera.resolution, dtype="uint8")
Csys=(300,389) #x,y coordinates 0,0 i venstre top
Dia=(500)
cv2.circle(circle, Csys,Dia,255,-1)



maskC = cv2.imread('test-img.png' , cv2.IMREAD_GRAYSCALE)
masked_data = cv2.bitwise_and(circle,maskC)
thresh1, binaryImg= cv2.threshold(masked_data,B,W,cv2.THRESH_BINARY_INV) #kørere et binært tærskel maske over grayscale billede



cv2.imwrite('/home/pi/Desktop/pakning image/test.png',binaryImg)
            
