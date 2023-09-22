import RPi.GPIO as GPIO
import time


vib_pin = 12
freq_pin = 21
enable_pin = 16
motor_pin = 18
ir_pin = 23

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(vib_pin,GPIO.OUT)
GPIO.setup(motor_pin,GPIO.OUT)
GPIO.setup(freq_pin,GPIO.IN)
GPIO.setup(enable_pin,GPIO.IN)
GPIO.setup(ir_pin,GPIO.IN)
#speed = GPIO.PWM(motor_pin, 60)
#speed.start(85)
pwm = GPIO.PWM(vib_pin, 29) # 50 Hz (20 ms PWM period)
pwm.start(50)
counter = 0
while(True):
    ir=GPIO.input(ir_pin) #6K ohm resistor + photoresistor at 3,3 V
    print(ir,counter)
    time.sleep(0.01)
    counter = (counter+ir*2)-1
        
    if(counter > 10):
        pwm.ChangeDutyCycle(0)
    if(counter >20):
        counter=20
    if(counter <= 0):
        counter=0
    elif(counter <=10):
        pwm.ChangeDutyCycle(50)

    




