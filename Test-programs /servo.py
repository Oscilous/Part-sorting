import RPi.GPIO as GPIO
import time

# setup the GPIO pin for the servo
kanp_pin = 19
servo_pin = 13
led_pin = 12
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin,GPIO.OUT)
GPIO.setup(kanp_pin,GPIO.IN)
GPIO.setup(led_pin,GPIO.OUT)
# setup PWM process
pwm = GPIO.PWM(servo_pin,50) # 50 Hz (20 ms PWM period)

lys = GPIO.PWM(led_pin,10000) # 50 Hz (20 ms PWM period)


lys.start(0.01)
pwm.start(6.8) # start PWM by rotating to 90 degrees

for ii in range(0,1):
    pwm.ChangeDutyCycle(2.0) # rotate to 0 degrees
    time.sleep(0.5)
    pwm.ChangeDutyCycle(10.0) # rotate to 180 degrees
    time.sleep(0.5)
    pwm.ChangeDutyCycle(7.0) # rotate to 90 degrees
    time.sleep(0.5)

pwm.ChangeDutyCycle(0) # this prevents jitter
pwm.stop() # stops the pwm on 13
GPIO.cleanup() # good practice when finished using a pin

