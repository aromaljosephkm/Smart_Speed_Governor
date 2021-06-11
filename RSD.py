import time
import RPi.GPIO as GPIO
import threading
import os
from scipy.spatial import distance as dist
from imutils.video import VideoStream
from imutils import face_utils
from threading import Thread
import numpy as np
import playsound
import imutils
import dlib
import cv2

GPIO.setwarnings(False)
GPIO.cleanup()
GPIO.setmode(GPIO.BOARD)

GPIO_TRIGGER = 7
GPIO_ECHO    = 12
GPIO_GREEN   = 11
GPIO_YELLOW  = 13
GPIO_RED     = 15
GPIO_MotorB  = 18
GPIO_MotorE  = 32
GPIO_Temp    = 40

GPIO.setup(GPIO_GREEN, GPIO.OUT)
GPIO.setup(GPIO_YELLOW, GPIO.OUT)
GPIO.setup(GPIO_RED, GPIO.OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
GPIO.setup(GPIO_MotorA,GPIO.OUT)
GPIO.setup(GPIO_MotorB,GPIO.OUT)
GPIO.setup(GPIO_MotorE,GPIO.OUT)
GPIO.setup(GPIO_Temp, GPIO.OUT)
pwm=GPIO.PWM(GPIO_MotorE,10)

def changeDuty(duty):
    pwm.ChangeDutyCycle(duty)

def motorRun():
    pwm.start(50)
    GPIO.output(GPIO_MotorA,GPIO.HIGH)
    GPIO.output(GPIO_MotorB,GPIO.LOW)
    GPIO.output(GPIO_MotorE,GPIO.HIGH)

def tempCheck():
    while True:
        x = os.popen('vcgencmd measure_temp').read()
        output = x[5:7]
        if int(output)>40:
            GPIO.output(GPIO_Temp, GPIO.HIGH)
        else:
            GPIO.output(GPIO_Temp, GPIO.LOW)

def greenLight():
    changeDuty(50)
    GPIO.output(GPIO_GREEN, GPIO.HIGH)
    GPIO.output(GPIO_YELLOW, GPIO.LOW)
    GPIO.output(GPIO_RED, GPIO.LOW)

def yellowLight():
    changeDuty(35)
    GPIO.output(GPIO_YELLOW, GPIO.HIGH)
    GPIO.output(GPIO_GREEN, GPIO.LOW)
    GPIO.output(GPIO_RED, GPIO.LOW)

def redLight():
    changeDuty(20)
    GPIO.output(GPIO_RED, GPIO.HIGH)
    GPIO.output(GPIO_GREEN, GPIO.LOW)
    GPIO.output(GPIO_YELLOW, GPIO.LOW)   

def getDistance():
    GPIO.output(GPIO_TRIGGER, True)
    time.sleep(0.0001)
    GPIO.output(GPIO_TRIGGER, False)
    while GPIO.input(GPIO_ECHO) == False:
        start = time.time()
    while GPIO.input(GPIO_ECHO) == True:
        stop = time.time()
    elapsed = stop - start
    distance = elapsed / 0.000058
    return distance

def speedGovernor():
    while True:
        distance = getDistance()
        time.sleep(0.05)
        print(distance)

        if distance > 20:
            greenLight()
            print("Long Distance Ahead...")
            time.sleep(0.5)
        elif 20 >= distance > 10:
            yellowLight()
            print("Medium Distance Ahead...")
            time.sleep(0.5)
        else:
            redLight()
            print("Low Distance Ahead...")
            time.sleep(0.5)

def sound_alarm(path):
	playsound.playsound(path)

def eye_aspect_ratio(eye):
	A = dist.euclidean(eye[1], eye[5])
	B = dist.euclidean(eye[2], eye[4])
	C = dist.euclidean(eye[0], eye[3])
	ear = (A + B) / (2.0 * C)
	return ear

def drowsinessChecker():
    shapePredictor = "shape_predictor_68_face_landmarks.dat"
    alarmFile = "alarm.WAV"
    webcam = 0

    EYE_AR_THRESH = 0.3
    EYE_AR_CONSEC_FRAMES = 48

    COUNTER = 0
    ALARM_ON = False

    print("[INFO] loading facial landmark predictor...")
    detector = dlib.get_frontal_face_detector()
    predictor = dlib.shape_predictor(shapePredictor)

    (lStart, lEnd) = face_utils.FACIAL_LANDMARKS_IDXS["left_eye"]
    (rStart, rEnd) = face_utils.FACIAL_LANDMARKS_IDXS["right_eye"]

    print("[INFO] starting video stream thread...")
    vs = VideoStream(src=webcam).start()
    time.sleep(1.0)

    while True:

        frame = vs.read()
        frame = imutils.resize(frame, width=450)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        rects = detector(gray, 0)

        for rect in rects:

            shape = predictor(gray, rect)
            shape = face_utils.shape_to_np(shape)

            leftEye = shape[lStart:lEnd]
            rightEye = shape[rStart:rEnd]
            leftEAR = eye_aspect_ratio(leftEye)
            rightEAR = eye_aspect_ratio(rightEye)

            ear = (leftEAR + rightEAR) / 2.0

            leftEyeHull = cv2.convexHull(leftEye)
            rightEyeHull = cv2.convexHull(rightEye)
            cv2.drawContours(frame, [leftEyeHull], -1, (0, 255, 0), 1)
            cv2.drawContours(frame, [rightEyeHull], -1, (0, 255, 0), 1)

            if ear < EYE_AR_THRESH:
                COUNTER += 1

                if COUNTER >= EYE_AR_CONSEC_FRAMES:

                    if not ALARM_ON:
                        ALARM_ON = True

                        if alarmFile != "":
                            t = Thread(target=sound_alarm,
                                args=(alarmFile,))
                            t.deamon = True
                            t.start()

                    cv2.putText(frame, "DROWSINESS ALERT!", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            else:
                COUNTER = 0
                ALARM_ON = False

            cv2.putText(frame, "Ratio: {:.2f}".format(ear), (300, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break

    cv2.destroyAllWindows()
    vs.stop()

if __name__ == "__main__":
    print("System Starting Up...")
    greenLight()
    time.sleep(1)
    yellowLight()
    time.sleep(1)
    redLight()
    time.sleep(1)

    t1 = threading.Thread(target=tempCheck, args=())
    t1.start()
    motorRun()
    t2 = threading.Thread(target=speedGovernor, args=())
    t2.start()
    t3 = threading.Thread(target=drowsinessChecker, args=())
    t3.start()
