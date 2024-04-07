# import raspberry pi GPIO module
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
# setup pin 3 as output
forwR = 13  # prawe przod BCM 27 Board 13
backR = 11  # prawe tyl BCM 17 Board 11
pwmB = 15  # pwm prawe BCM 22 Board 15

forwL = 18  # lewe przod BCM 24 Board 18
backL = 16  # lewe tyl BCM 23 Board 16
pwmA = 22  # pwm lewe BCM 25 Board 22

baseL = 60
baseR = 60

GPIO.setup(forwR, GPIO.OUT)  # prawe przod
GPIO.setup(backR, GPIO.OUT)  # prawe tyl
GPIO.setup(pwmB, GPIO.OUT)  # pwm prawe

GPIO.setup(forwL, GPIO.OUT)  # lewe przod
GPIO.setup(backL, GPIO.OUT)  # lewe tyl
GPIO.setup(pwmA, GPIO.OUT)  # pwm lewe

# setup pin 3 as pulse width modulation output
# frequency = 2 hz (2 times per second)
pwmR = GPIO.PWM(pwmB, 200)
pwmL = GPIO.PWM(pwmA, 200)
GPIO.output(forwR, GPIO.LOW)
GPIO.output(backR, GPIO.LOW)
GPIO.output(forwL, GPIO.LOW)
GPIO.output(backL, GPIO.LOW)
pwmL.start(baseL)
pwmR.start(baseR)


def move_forward(t=0):
    GPIO.output(forwR, GPIO.HIGH)
    GPIO.output(forwL, GPIO.HIGH)
    if t != 0:
        time.sleep(t)
        GPIO.output(forwR, GPIO.LOW)
        GPIO.output(forwL, GPIO.LOW)


def move_backward(t=0):
    GPIO.output(backR, GPIO.HIGH)
    GPIO.output(backL, GPIO.HIGH)
    if t != 0:
        time.sleep(t)
        GPIO.output(backR, GPIO.LOW)
        GPIO.output(backL, GPIO.LOW)


def turn_left(t=0):
    GPIO.output(forwL, GPIO.HIGH)
    GPIO.output(backR, GPIO.HIGH)
    if t != 0:
        time.sleep(t)
        GPIO.output(forwL, GPIO.LOW)
        GPIO.output(backR, GPIO.LOW)


def turn_right(t=0):
    GPIO.output(backL, GPIO.HIGH)
    GPIO.output(forwR, GPIO.HIGH)
    if t != 0:
        time.sleep(t)
        GPIO.output(backL, GPIO.LOW)
        GPIO.output(forwR, GPIO.LOW)


def stop():
    GPIO.output(forwR, GPIO.LOW)
    GPIO.output(forwL, GPIO.LOW)
    GPIO.output(backL, GPIO.LOW)
    GPIO.output(backR, GPIO.LOW)

