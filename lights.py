import RPi.GPIO as GPIO
from time import sleep

GPIO.setwarnings(False)
GPIO.cleanup()
GPIO.setmode(GPIO.BOARD)

GPIO_GREEN   = 11
GPIO_YELLOW  = 13
GPIO_RED     = 15

GPIO.setup(GPIO_GREEN, GPIO.OUT)
GPIO.setup(GPIO_YELLOW, GPIO.OUT)
GPIO.setup(GPIO_RED, GPIO.OUT)

GPIO.output(GPIO_RED, GPIO.HIGH)
sleep(5)
GPIO.output(GPIO_RED, GPIO.LOW)
GPIO.output(GPIO_YELLOW, GPIO.HIGH)
sleep(5)
GPIO.output(GPIO_YELLOW, GPIO.LOW)
GPIO.output(GPIO_GREEN, GPIO.HIGH)
sleep(5)
GPIO.output(GPIO_GREEN, GPIO.LOW)