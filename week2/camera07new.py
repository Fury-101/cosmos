# This program illustrates how to capture a images as part of a video stream
# and how to do extract pixels of a specific color
# It includes a slider to adjust the color that is being filtered
# It uses openCV

import cv2
import picamera
import picamera.array                                   # This needs to be imported explicitly
import time
import numpy as np
import RPi.GPIO as GPIO

# GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BOARD)
 
# set GPIO Pins
GPIO_Ain1 = 11
GPIO_Ain2 = 13
GPIO_Apwm = 15
GPIO_Bin1 = 29
GPIO_Bin2 = 31
GPIO_Bpwm = 33

# Set GPIO direction (IN / OUT)
GPIO.setup(GPIO_Ain1, GPIO.OUT)
GPIO.setup(GPIO_Ain2, GPIO.OUT)
GPIO.setup(GPIO_Apwm, GPIO.OUT)
GPIO.setup(GPIO_Bin1, GPIO.OUT)
GPIO.setup(GPIO_Bin2, GPIO.OUT)
GPIO.setup(GPIO_Bpwm, GPIO.OUT)

# Both motors are stopped 
GPIO.output(GPIO_Ain1, False)
GPIO.output(GPIO_Ain2, False)
GPIO.output(GPIO_Bin1, False)
GPIO.output(GPIO_Bin2, False)

# Set PWM parameters
pwm_frequency = 50

# Create the PWM instances
pwmA = GPIO.PWM(GPIO_Apwm, pwm_frequency)
pwmB = GPIO.PWM(GPIO_Bpwm, pwm_frequency)

# Set the duty cycle (between 0 and 100)
# The duty cycle determines the speed of the wheels
pwmA.start(100)
pwmB.start(100)

def setspeed(right_speed, left_speed):
    GPIO.output(GPIO_Ain1, True ^ (right_speed < 0))
    GPIO.output(GPIO_Ain2, False ^ (right_speed < 0))
    pwmA.ChangeDutyCycle(abs(right_speed))
    GPIO.output(GPIO_Bin1, True ^ (left_speed < 0))
    GPIO.output(GPIO_Bin2, False ^ (left_speed < 0))
    pwmB.ChangeDutyCycle(abs(left_speed))
    
# Initialize the camera and grab a reference to the frame
camera = picamera.PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32


CAM_SIZE = (640, 480)
# Create an array to store a frame
rawframe = picamera.array.PiRGBArray(camera, size=CAM_SIZE)


print("Press CTRL+C to end the program.")
# white filter:  result1: 0 0 17 result 2: 180 27 255
# yellow:
colors =  {
    "white": {
        "lower": [0, 0, 17],
        "upper": [180, 27, 255]
    },
    "pink": { 
        "lower": [152, 134, 91],
        "upper": [175, 255, 231]
    },
    "orange": { # TURN RIGHT
        "lower": [3, 45, 52],
        "upper": [10, 254, 255]
    },
    "yellow": { 
        "lower": [14, 47, 143],
        "upper": [35, 219, 231]
    },
    "green": { 
        "lower": [44, 47, 119],
        "upper": [70, 212, 231]
    },
    "blue": { # TURN LEFTß
        "lower": [71, 20, 22],
        "upper": [106, 255, 231]
    },
#     "indigo": {
#         "lower": [0, 76, 143],
#         "upper": [17, 219, 231]
#     },
    "purple": { 
        "lower": [122, 113, 36],
        "upper": [152, 255, 255]
    }
}
SPEED = 100
SPINSPEED = SPEED
COLOR_THRESH = 100

'''
STATE 0: Currently following currcolor
STATE 1: Following orange
STATE 2: Turning
'''

FSMState = 0

currcolor = "green"
nextcolor = "green" # "yellow"

setspeed(0, 0)
try:

        # allow the camera to warm up
        time.sleep(0.1)

        for frame in camera.capture_continuous(rawframe, format = "bgr", use_video_port = True):
            print(FSMState)

            image = frame.array
            image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # The colors in range are set to white (255), while the colors not in range are set to black (0)
            cmask = cv2.inRange(image_hsv,
                np.array(colors[currcolor]["lower"]),
                np.array(colors[currcolor]["upper"])
            )
            omask = cv2.inRange(image_hsv,
                np.array(colors["orange"]["lower"]),
                np.array(colors["orange"]["upper"])
            )
            bmask = cv2.inRange(image_hsv,
                np.array(colors["blue"]["lower"]),
                np.array(colors["blue"]["upper"])
            )
            cv2.imshow("Frame", image)
            cv2.imshow("cMask", cmask)
            cv2.imshow("oMask", omask)
            cv2.imshow("bMask", bmask)
            #kernel = np.array([[-1,-1,-1],[-1,8,-1],[-1,-1,-1]])		# outline
             
            #gmask = cv2.filter2D(gmask,-1,kernel) # Convolutional filter

            numpixelsOrange = cv2.countNonZero(omask)
            numpixelsBlue = cv2.countNonZero(bmask)
            numpixels = cv2.countNonZero(cmask)
            breakFlag = False
            print("orange pixels", numpixelsOrange)
            print("blue pixels", numpixelsBlue)
            print(currcolor, "pixels", numpixels)
            if FSMState == 0 and numpixelsOrange > COLOR_THRESH:
                tmp = currcolor
                currcolor = nextcolor
                nextcolor = tmp
                FSMState = 1
                setspeed(0, 0)
                breakFlag = True
            elif FSMState == 0 and numpixelsBlue > COLOR_THRESH:
                tmp = currcolor
                currcolor = nextcolor
                nextcolor = tmp
                FSMState = 3
                setspeed(0, 0)
                breakFlag = True
            elif FSMState == 1 and numpixelsOrange == 0:
                FSMState = 2
                setspeed(0, 0)
                breakFlag = True
            elif FSMState == 3 and numpixelsBlue == 0:
                FSMState = 4
                setspeed(0, 0)
                breakFlag = True

            if (not breakFlag):
                breakFlag = False
                if (FSMState == 0 and numpixels == 0):
                    setspeed(SPEED, SPEED)
                    print("i can't see anything but i'm going forward")
                    breakFlag = True
                if FSMState == 2:
                    if (numpixels > COLOR_THRESH):
                        setspeed(0, 0)
                        FSMState = 0
                    else:
                        setspeed(SPINSPEED, -SPINSPEED) 
                    breakFlag = True
                elif FSMState == 4:
                    if (numpixels > COLOR_THRESH):
                        setspeed(0, 0)
                        FSMState = 0
                    else:
                        setspeed(0, 2*SPINSPEED) # setspeed(-SPINSPEED, SPINSPEED)
                    breakFlag = True

                if (not breakFlag):
                    if FSMState == 0:
                        image_masked = cv2.bitwise_and(image, image, mask = cmask)
                    
                        contours = cv2.findContours(cmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    elif FSMState == 1:
                        image_masked = cv2.bitwise_and(image, image, mask = omask)
                    
                        contours = cv2.findContours(omask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    elif FSMState == 3:
                        image_masked = cv2.bitwise_and(image, image, mask = bmask)
                    
                        contours = cv2.findContours(bmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        
                    cv2.drawContours(image_masked, contours[0], -1, (255, 0, 0), 2)
                    cx = None
                    cy = None
                    for i, c in enumerate(contours[0]):
                        M = cv2.moments(c)
                        if M["m00"] != 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                            
                            cv2.circle(image_masked, (cx, cy), 5, (255, 255, 255), -1)
                            break
                        
                    if cx is not None and cy is not None:
                        left_speed = cx / CAM_SIZE[0] * SPEED
                        right_speed = SPEED - left_speed
                    else:
                        left_speed = 0
                        right_speed = 0
                        
                    setspeed(left_speed, right_speed)

                    cv2.imshow("Result", image_masked)       
                    cv2.waitKey(1)
                    
            cv2.waitKey(1)
            # Clear the stream in preparation for the next frame
            rawframe.truncate(0)

        
# Quit the program when the user presses CTRL + C
except (KeyboardInterrupt, picamera.exc.PiCameraValueError):
        # Clean up the resources
        cv2.destroyAllWindows()
        camera.close()
        GPIO.cleanup()
        pwmA.stop()
        pwmB.stop()