import time
import RPi.GPIO as GPIO
import _thread
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
GPIO.setmode(GPIO.BCM)

GPIO_TRIGGER = 4
GPIO_ECHO    = 18
GPIO_GREEN   = 17
GPIO_YELLOW  = 27
GPIO_RED     = 22
GPIO_IN_1    = 23
GPIO_IN_2    = 24

GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
GPIO.setup(GPIO_GREEN, GPIO.OUT)
GPIO.setup(GPIO_YELLOW, GPIO.OUT)
GPIO.setup(GPIO_RED, GPIO.OUT)
GPIO.setup(GPIO_IN_1, GPIO.OUT)
GPIO.setup(GPIO_IN_2, GPIO.OUT)

def greenLight():
    GPIO.output(GPIO_GREEN, GPIO.HIGH)
    GPIO.output(GPIO_YELLOW, GPIO.LOW)
    GPIO.output(GPIO_RED, GPIO.LOW)

def yellowLight():
    GPIO.output(GPIO_YELLOW, GPIO.HIGH)
    GPIO.output(GPIO_GREEN, GPIO.LOW)
    GPIO.output(GPIO_RED, GPIO.LOW)

def redLight():
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

    distance = elapsed * 0.000058

    print("Distance : {} cm".format(distance))
    return distance

def speedGovernor():
    while True:
        distance = getDistance()
        time.sleep(0.05)

        if distance > 50:
            greenLight()
            motorController(8)
        elif 50 <= distance >= 20:
            yellowLight()
            motorController(5)
        elif distance < 20:
            redLight()
            motorController(2)
        elif distance < 5:
            redLight()
            motorController(0)

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
    alarmFile = "alarm.wav"
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

def set(property, value):
    try:
        f = open("/sys/class/rpi-pwm/pwm0/" + property, 'w')
        f.write(value)
        f.close()	
    except:
        print("Error writing to: " + property + " value: " + value)

def clockwise():
    GPIO.output(GPIO_IN_1, True)    
    GPIO.output(GPIO_IN_2, False)

'''def counter_clockwise():
    GPIO.output(GPIO_IN_1, False)
    GPIO.output(GPIO_IN_2, True)'''

def motorController(cmd):
    while True:
        '''cmd = input("Command, f/r 0..9, E.g. f5 :")
        direction = cmd[0]
        if direction == "f":
            clockwise()
        else: 
            counter_clockwise()
        speed = int(cmd[1]) * 11'''
        speed = int(cmd) * 11
        set("duty", str(speed))


if __name__ == "__main__":
    set("delayed", "0")
    set("mode", "pwm")
    set("frequency", "500")
    set("active", "1")
    clockwise()
    _thread.start_new_thread(speedGovernor, ())
    _thread.start_new_thread(drowsinessChecker, ())
