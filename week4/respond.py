from gtts import gTTS
import speech_recognition as sr
from datetime import datetime
import pygame
import os
import random

import cv2
import numpy as np
import picamera
import picamera.array                                   # This needs to be imported explicitly
import time
import numpy as np
import RPi.GPIO as GPIO
from adafruit_servokit import ServoKit

pygame.mixer.init()

# PHYSICAL SETUP
kit = ServoKit(channels=16)

'''
Servo 0: Left arm (left/right) 
Servo 1: Right arm (up/down)
Servo 2: Head up/down
Servo 3: Head left/right
Continuous servo 4: Spin Rock
Continuous servo 5: Spin Paper
Continuous servo 6: Spin Scissors
'''

RANGE_OF_MOTION = 45
def moveHand(dir):
    if dir == 'up':
        kit.servo[1].angle = 90 + RANGE_OF_MOTION
    elif dir == 'down':
        kit.servo[1].angle = 90 - RANGE_OF_MOTION
    elif dir == 'left':
        kit.servo[0].angle = 90 + RANGE_OF_MOTION
    elif dir == 'right':
        kit.servo[0].angle = 90 - RANGE_OF_MOTION
    elif dir == 'reset':
        kit.servo[0].angle = 90
        kit.servo[1].angle = 90
def moveHead(dir):
    if dir == 'up':
        kit.servo[2].angle = 135 + RANGE_OF_MOTION
    elif dir == 'down':
        kit.servo[2].angle = 135 - RANGE_OF_MOTION
    elif dir == 'left':
        kit.servo[3].angle = 135 + RANGE_OF_MOTION
    elif dir == 'right':
        kit.servo[3].angle = 135 - RANGE_OF_MOTION
    elif dir == 'reset':
        kit.servo[2].angle = 135
        kit.servo[3].angle = 135

# GPIO Mode (BOARD / BCM)
# GPIO.setmode(GPIO.BOARD)
 
# set GPIO Pins
GPIO_Ain1 = 17
GPIO_Ain2 = 27
GPIO_Apwm = 22
GPIO_Bin1 = 5
GPIO_Bin2 = 6
GPIO_Bpwm = 13

GPIO.setup(GPIO_Ain1, GPIO.OUT)
GPIO.setup(GPIO_Ain2, GPIO.OUT)
GPIO.setup(GPIO_Apwm, GPIO.OUT)
GPIO.setup(GPIO_Bin1, GPIO.OUT)
GPIO.setup(GPIO_Bin2, GPIO.OUT)
GPIO.setup(GPIO_Bpwm, GPIO.OUT)

pwm_frequency = 50

pwmA = GPIO.PWM(GPIO_Apwm, pwm_frequency)
pwmB = GPIO.PWM(GPIO_Bpwm, pwm_frequency)

pwmA.start(100)
pwmB.start(100)

def setspeed(right_speed, left_speed):
    GPIO.output(GPIO_Ain1, False ^ (right_speed < 0))
    GPIO.output(GPIO_Ain2, True ^ (right_speed < 0))
    pwmA.ChangeDutyCycle(abs(right_speed))
    GPIO.output(GPIO_Bin1, False ^ (left_speed < 0))
    GPIO.output(GPIO_Bin2, True ^ (left_speed < 0))
    pwmB.ChangeDutyCycle(abs(left_speed))
setspeed(0, 0)

camera = picamera.PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32

CAM_SIZE = (640, 480)
# Create an array to store a frame
rawframe = picamera.array.PiRGBArray(camera, size=CAM_SIZE)

NAME = 'dumbo'
SPEED = 100
SPINSPEED = SPEED // 3
COLORTHRESH = 1000

backhandColor = { # red 
    "lower": [166, 176, 67],
    "upper": [180, 255, 227]
}
forehandColor = { # pink
    "lower": [141, 74, 155],
    "upper": [156, 167, 227]
}

'''
STATE 0: Idling -- listening for commands
STATE 1: Searching for forehand color
STATE 2: Shadow boxing
STATE 3: Playing Rock Paper Scissors
'''
mainstate = 2

# SPEECH VARIABLES
r = sr.Recognizer()
words = []
def speak(txt, lang='en', tld='us'):
    filename = datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ".mp3"
    tts = gTTS(txt, lang=lang, tld=tld)
    tts.save(filename)
    pygame.mixer.music.load(filename)
    pygame.mixer.music.play()
    os.remove(filename)

# SHADOWBOXING VARIABLES
'''
TURN True: Bot's turn
TURN False: Opponent's turn
'''
HAND_MOVEMENT_THRESH = 100
handMoved = False
turn = True
possibleDirs = ['up', 'down', 'right', 'left']
previousmoves = []
direction = None
initialNose = (None, None)
initialHand = (None, None)
accumulatedDiff = np.zeros(2)
model = cv2.FaceDetectorYN.create(
    model="./face_detection_yunet_2022mar.onnx",
    config="",
    input_size=CAM_SIZE,
    top_k=5000
)
def getNose(img):
    _, faces = model.detect(img)
    if faces is None:
        return (None, None)
    
    for face in faces:
        coords = face[:-1].astype(np.int32)

    return (coords[8], coords[9])

def getDir(accumulated): # where accumulated is [x, y]
    x, y = accumulated
    if abs(x) > abs(y):

        return 'right' if x > 0 else 'left'
    return 'down' if y > 0 else 'up'
    

# RPS VARIABLES
RPS_CONSTANT = 100
lastTime = None
choice = None
counter = 1

def reset():
    global turn, possibleDirs, previousmoves, direction, initialNose, accumulatedDiff, words, lastTime, choice, counter, handMoved
    turn = True
    possibleDirs = ['up', 'down', 'right', 'left']
    previousmoves = []
    direction = None
    initialNose = (None, None)
    accumulatedDiff = np.zeros(2)

    words = []
    lastTime = None
    choice = None
    counter = 1
    handMoved = False
    setspeed(0, 0)
    moveHand('reset')
    moveHead('reset')
    for i in range(4, 7):
        kit.continuous_servo[i].throttle = 0
reset()

def findCenter(img):
    contours = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cx = None
    cy = None
    for i, c in enumerate(contours[0]):
        M = cv2.moments(c)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            break
    
    return (cx, cy)

AREA_PERIM_RATIO = 0.1
with sr.Microphone() as source:
    r.adjust_for_ambient_noise(source)
    # allow camera to warm up
    time.sleep(0.1)
    # speak("Ready to rumble")
    for frame in camera.capture_continuous(rawframe, format = "bgr", use_video_port = True):
        image = frame.array
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        if mainstate == 0:
            print("Say something!")
            audio = r.listen(source,timeout=5,phrase_time_limit=2)
            try:
                txt = r.recognize_google(audio).lower()
                print("Heard", txt)
                tkns = txt.split()
                words.extend(tkns)
            except (LookupError, sr.exceptions.UnknownValueError):
                print("Could not understand audio")

            print(words)
            cmd = None # command as a list of words
            greetings = ["hey", "play", "yo"]
            for i in range(len(words) - 1):
                if words[i] in greetings and (words[i+1] == NAME or words[i+1] == "dumb" or words[i+1] == "jumbo"):
                    if (i+2 < len(words)):
                        cmd = words[i+2:]
                    words = words[i:]
                    break
            print("Following command", cmd)

            if cmd is not None:
                if "come" in cmd or "follow" in cmd:
                    speak("On my way!")
                    mainstate = 1
                    reset()
                elif ("shadow" in cmd and "box" in cmd) or "shutterbox" in cmd or "otterbox" in cmd:
                    speak("Shadow boxing? I'll go first!")
                    mainstate = 2
                    reset()
                    time.sleep(5)
                elif "rock" in cmd and "scissors" in cmd and "paper" in cmd:
                    speak("Rock paper scissors? Sure!")
                    mainstate = 3
                    reset()
                    time.sleep(5)
        elif mainstate == 1:
            fmask = cv2.inRange(image_hsv,
                np.array(forehandColor["lower"]),
                np.array(forehandColor["upper"])
            )

            cx, cy = findCenter(fmask)
                
            if cx is not None and cy is not None:
                left_speed = cx / CAM_SIZE[0] * SPEED
                right_speed = SPEED - left_speed
                print("Found color, going towards it --")
            else:
                left_speed = -SPINSPEED
                right_speed = SPINSPEED
                print("Spinning")

            setspeed(right_speed, left_speed)

            bmask = cv2.inRange(image_hsv,
                np.array(backhandColor["lower"]),
                np.array(backhandColor["upper"])
            )

            contours, hierarchy = cv2.findContours(bmask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            ratioFlag = False
            for cnt in contours:
                area = cv2.contourArea(cnt, True)
                perim = cv2.arcLength(cnt, True)
                if perim == 0: continue
                if area / perim > AREA_PERIM_RATIO:
                    ratioFlag = True

            numpixelsBackhand = cv2.countNonZero(bmask)
            if numpixelsBackhand > COLORTHRESH and ratioFlag:
                print("Stopping because I saw the backhand -- ", numpixelsBackhand)
                speak("Hello there")
                mainstate = 0
                reset()
        elif mainstate == 2:
            if turn:
                if direction is None:
                    if len(possibleDirs) == 1:
                        speak("I won! Better luck next time.")
                        reset()
                        mainstate = 0
                    else:
                        for pmove in previousmoves:
                            moveHand(pmove)
                            time.sleep(0.25)
                            moveHand('reset')
                        direction = random.choice(possibleDirs)
                        print("Going (hand)", direction, "out of", possibleDirs)
                        possibleDirs.remove(direction)
                        previousmoves.append(direction)

                        time.sleep(1)
                        moveHand(direction)
                        initialNose = getNose(image)
                        if initialNose == (None, None):
                            speak("Sorry, I couldn't see your face!")
                            mainstate = 0
                            reset()
                        lastTime = time.time()
                if mainstate == 2:
                    nose = getNose(image)
                    if nose != (None, None):
                        diff = np.subtract(nose, initialNose)
                        accumulatedDiff += diff
                    if time.time() - lastTime > 1:
                        dir = getDir(accumulatedDiff)
                        print("Opponent moved their head", dir)
                        if dir == direction:
                            direction = None
                            lastTime = None
                            initialNose = (None, None)
                            accumulatedDiff = [0, 0]
                            moveHand('reset')
                            print("Got a hit")
                            speak("Gotcha!")
                            time.sleep(1)
                        else:
                            reset()
                            turn = False
                            print("I missed my hit")
                            time.sleep(1)               
            else:
                fmask = cv2.inRange(image_hsv,
                    np.array(forehandColor["lower"]),
                    np.array(forehandColor["upper"])
                )
                bmask = cv2.inRange(image_hsv,
                    np.array(backhandColor["lower"]),
                    np.array(backhandColor["upper"])
                )
                mask = cv2.bitwise_or(fmask, bmask)

                if direction is None:
                    if len(possibleDirs) == 1:
                        speak("Congrats, you won!")
                        reset()
                        mainstate = 0
                    else:
                        direction = random.choice(possibleDirs)
                        possibleDirs.remove(direction)
                        previousmoves.append(direction)
                                                
                        cx, cy = findCenter(mask)
                        
                        if cx is None and cy is None:
                            speak("Sorry, I couldn't find your hand!")
                            mainstate = 0
                            reset()
                        
                        lastTime = time.time()
                        initialHand = (cx, cy)

                if mainstate == 2:
                    center = findCenter(mask)

                    if center == (None, None):
                        speak("Sorry, I couldn't find your hand!")
                        mainstate = 0
                        reset()
                    else:    
                        diff = np.subtract(center, initialHand)
                        accumulatedDiff += diff
                        if (sum(diff * diff)**0.5 > HAND_MOVEMENT_THRESH and not handMoved):
                            for pmove in previousmoves:
                                moveHead(pmove)
                                time.sleep(0.25)
                                moveHead('reset')
                            time.sleep(1)
                            print("Going (head)", direction, "out of", possibleDirs)
                            moveHead(direction)
                            handMoved = True
                    
                    if time.time() - lastTime > 1:
                        dir = getDir(accumulatedDiff)
                        print("Opponent swung", dir)
                        if dir == direction:
                            direction = None
                            lastTime = None
                            initialHand = None
                            handMoved = False
                            accumulatedDiff = [0, 0]
                            moveHead('reset')
                            print("Opponent got a hit")
                            time.sleep(1)
                        else:
                            reset()
                            turn = True
                            print("Opponent missed their hit")
                            time.sleep(1)
        elif mainstate == 3:
            if lastTime is None:
                choice = random.choice(['rock', 'paper', 'scissors'])
                lastTime = time.time()
                speak("Rock")
            elif time.time() - lastTime > 1 and counter < 3:
                if 3 - counter == 1:
                    speak("Paper")
                elif 3 - counter == 2:
                    speak("Scissors")
                counter += 1
                lastTime = time.time()
            else:
                speak("Shoot")

                if choice == 'rock':
                    kit.continuous_servo[4].throttle = 1
                elif choice == 'paper':
                    kit.continuous_servo[5].throttle = 1
                elif choice == 'scissors':
                    kit.continuous_servo[6].throttle = 1

                if time.time() - lastTime > 1:
                    fmask = cv2.inRange(image_hsv,
                        np.array(forehandColor["lower"]),
                        np.array(forehandColor["upper"])
                    )
                    bmask = cv2.inRange(image_hsv,
                        np.array(backhandColor["lower"]),
                        np.array(backhandColor["upper"])
                    )

                    numpixelsForehand = cv2.countNonZero(fmask)
                    numpixelsBackhand = cv2.countNonZero(bmask)

                    opponent = None
                    if abs(numpixelsForehand - numpixelsBackhand) < RPS_CONSTANT:
                        opponent = 'scissors'
                    elif numpixelsBackhand <= RPS_CONSTANT:
                        opponent = 'paper'
                    elif numpixelsForehand <= RPS_CONSTANT:
                        opponent = 'rock'
                    else:
                        opponent = 'scissors'
                        print("Assumed scissors")

                    if opponent is not None:
                        won = (opponent == 'scissors' and choice == 'rock') \
                        or (opponent == 'rock' and choice == 'paper') \
                        or (opponent == 'paper' and choice == 'scissors')

                        speak(f"I picked {choice}! " + ("Congrats, you won!" if won else "Better luck next time."))   
                        lastTime = None
                        choice = None
                        time.sleep(2)
                        for i in range(4, 7):
                            kit.continuous_servo[i].throttle = 0

        
        # cv2.imshow("frame", image)
        # cv2.waitKey(1)

        rawframe.truncate(0)
