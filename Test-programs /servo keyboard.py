import cv2
import numpy as np
import RPi.GPIO as GPIO
import time



servo_pin = 13
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin,GPIO.OUT)

pwm = GPIO.PWM(servo_pin,50) # 50 Hz (20 ms PWM period)
pwm.start(6.8)
time.sleep(0.29)
pwm.ChangeDutyCycle(0)

#time.sleep(2)

pwm.ChangeDutyCycle(3)# rotate to 90 degrees
time.sleep(0.2)
pwm.ChangeDutyCycle(0)

#time.sleep(2)

pwm.ChangeDutyCycle(5.6) # rotate to 90 degrees
time.sleep(0.2)
pwm.ChangeDutyCycle(0)