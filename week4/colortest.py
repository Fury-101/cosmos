from gtts import gTTS
import speech_recognition as sr

import cv2
import numpy as np
import picamera
import picamera.array                                   # This needs to be imported explicitly
import time
import numpy as np


camera = picamera.PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32

CAM_SIZE = (640, 480)
# Create an array to store a frame
rawframe = picamera.array.PiRGBArray(camera, size=CAM_SIZE)

backhandColor = { # red 
    "lower": [166, 176, 67],
    "upper": [180, 255, 227]
}
forehandColor = { # pink
    "lower": [141, 74, 155],
    "upper": [156, 167, 227]
}
    
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

for frame in camera.capture_continuous(rawframe, format = "bgr", use_video_port = True):
    image = frame.array
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    fmask = cv2.inRange(image_hsv,
        np.array(forehandColor["lower"]),
        np.array(forehandColor["upper"])
    )
    bmask = cv2.inRange(image_hsv,
        np.array(backhandColor["lower"]),
        np.array(backhandColor["upper"])
    )
    mask = cv2.bitwise_or(fmask, bmask)
        
    cv2.imshow("frame", mask)
    cv2.waitKey(1)

    rawframe.truncate(0)