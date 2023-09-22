import cv2
import numpy as np
import imutils
from picamera.array import PiRGBArray
from picamera import PiCamera 
import RPi.GPIO as GPIO
import time



def color_seg(choice):
    if choice == 'blue':
        lower_hue = np.array([100,30,30])
        upper_hue = np.array([150,148,255])
    elif choice == 'white':
        lower_hue = np.array([0,0,0])
        upper_hue = np.array([0,0,255])
    elif choice == 'black':
        lower_hue = np.array([0,0,0])
        upper_hue = np.array([50,50,100])
    return lower_hue, upper_hue


camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 10

rawCapture = PiRGBArray(camera, size=(640, 480))



# Take each frame
frame = cv2.imread('images/black0.jpg')
#frame = cv2.imread('images/road_1.jpg')

frame = imutils.resize(frame, height = 300)
chosen_color = 'black'


# Convert BGR to HSV
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

# define range of a color in HSV
lower_hue, upper_hue = color_seg(chosen_color)


# Threshold the HSV image to get only blue colors
mask = cv2.inRange(hsv, lower_hue, upper_hue)



cv2.imshow('frame',frame)
cv2.imshow('mask',mask)

cv2.waitKey(0)
