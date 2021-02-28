import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.cleanup()
GPIO.setmode(GPIO.BOARD)

GPIO_TRIGGER = 7
GPIO_ECHO    = 12

GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

GPIO.output(GPIO_TRIGGER, True)
time.sleep(0.0001)
GPIO.output(GPIO_TRIGGER, False)
while GPIO.input(GPIO_ECHO) == False:
    start = time.time()
while GPIO.input(GPIO_ECHO) == True:
    stop = time.time()
elapsed = stop - start
distance = elapsed * 0.000058
print("Distance : {} cm".format(distance))