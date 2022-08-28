# RPiDistanceKeepingRobot python file. This project implements a distance keeping robot car with the help of python multiprocessing.
import RPi.GPIO as GPIO
import time as time
import signal as signal
 
def intHandler():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    sp.ChangeDutyCycle(0)
 
signal.signal(signal.SIGINT, intHandler)

#GPIO SETUP
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

PWM = 14
IN1 = 15
IN2 = 18

S_PWM = 23

GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(PWM, GPIO.OUT)
GPIO.setup(S_PWM, GPIO.OUT)

GPIO.output(IN1, GPIO.LOW)
GPIO.output(IN2, GPIO.HIGH)

p = GPIO.PWM(PWM, 1000)
sp = GPIO.PWM(S_PWM, 1000)
p.start(90)
sp.start(90)
time.sleep(100)
