# This program demonstrates the use of the PCA9685 PWM driver
# This is useful to effectively control multiple servos or motors
# In this example, there is a servo on channel 0 and channel 1
# The code also shows how you can control a motor (on channel 2)

# Libraries
import time
from adafruit_servokit import ServoKit
from evdev import InputDevice, categorize

# Initialize ServoKit for the PWA board.
kit = ServoKit(channels=16)

# GPIO library not necessary if we only use the PWM driver
#import RPi.GPIO as GPIO

 
# GPIO Mode (BOARD / BCM)
#GPIO.setmode(GPIO.BOARD)


print("Press CTRL+C to end the program.")
gamepad = InputDevice('/dev/input/event0')

def toggle(servo,newstate):
    if (servo == 0):
        if (newstate):
            kit.servo[servo].angle = 120
        else:
            kit.servo[servo].angle = 0
    elif (servo == 1):
        if (newstate):
            kit.servo[servo].angle = 120
        else:
            kit.servo[servo].angle = 0
    elif (servo == 2):
        if (newstate):
            kit.servo[servo].angle = 0
        else:
            kit.servo[servo].angle = 120
    elif (servo == 3):
        if (newstate):
            kit.servo[servo].angle = 0
        else:
            kit.servo[servo].angle = 120

sleep = 0.75
turnsleep = 0.35

keys = {
    "f": False,
    "l": False,
    "r": False,
    "a": False,
    "y": False
}

lastTime = time.time()
flag = False
turnState = 0

# Main program 
try:
    state = 'n'
    while True:
        
        newstick  = False
        newbutton  = False
        try:
            for event in gamepad.read():            # Use this option (and comment out the next line) to react to the latest event only
                #event = gamepad.read_one()         # Use this option (and comment out the previous line) when you don't want to miss any event
                eventinfo = categorize(event)
                if event.type == 1:
                        newbutton = True
                        codebutton  = eventinfo.scancode
                        valuebutton = eventinfo.keystate
                elif event.type == 3:
                    newstick = True
                    codestick  = eventinfo.event.code
                    valuestick = eventinfo.event.value
        except:
            pass
        if (newstick and codestick == 1 and valuestick == 0):
            keys['f'] = True
        elif newstick and codestick == 1 and valuestick == 128:
            keys['f'] = False
        if (newstick and codestick == 0 and valuestick == 0):
            keys['l'] = True
        elif (newstick and codestick == 0 and valuestick == 128):
            keys['l'] = False
            keys['r'] = False
        elif newstick and codestick == 0 and valuestick == 255:
            keys['r'] = True
        if (newstick and codestick == 1 and valuestick == 255):
            keys['f'] = False
            keys['l'] = False
            keys['r'] = False
            for i in range(4):
                toggle(i, 0)
        elif newbutton and codebutton == 305 and valuebutton == 1:
            keys['a'] = True
        elif newbutton and codebutton == 305 and valuebutton == 0:
            keys['a'] = False
        elif newbutton and codebutton == 307 and valuebutton == 1:
            keys['y'] = True
        elif newbutton and codebutton == 307 and valuebutton == 0:
            keys['y'] = False
        if keys['f'] and (time.time() - lastTime > sleep):
            if (flag):
                toggle(0,1)
                toggle(2,1)
                toggle(1,0)
                toggle(3,0)
            else:
                toggle(0,0)
                toggle(2,0)
                toggle(1,1)
                toggle(3,1)
            flag = not flag
            lastTime = time.time()
        elif keys['a'] and ((time.time() - lastTime) > turnsleep):
            if turnState == 0:
                toggle(0,0)
                toggle(1,0)
                toggle(2,0)
                toggle(3,0)
            elif turnState == 1:
                toggle(0,1)
                toggle(1,1)
                toggle(3,1)
            elif turnState == 2:
                toggle(0,1)
            turnState += 1
            turnState %= 3
            lastTime = time.time()
        elif keys['y'] and ((time.time() - lastTime) > turnsleep):
            if turnState == 0:
                toggle(0,0)
                toggle(1,0)
                toggle(2,0)
                toggle(3,0)
            elif turnState == 1:
                toggle(2,1)
                toggle(1,1)
                toggle(3,1)
            elif turnState == 2:
                toggle(2,1)
            turnState += 1
            turnState %= 3
            lastTime = time.time()
        elif keys['r'] and ((time.time() - lastTime) > turnsleep):
            if turnState == 0:
                toggle(0,0)
                toggle(1,0)
                toggle(2,1)
                toggle(3,1)
            elif turnState == 1:
                toggle(0,1)
                toggle(1,0)
            elif turnState == 2:
                toggle(1,1)
            elif turnState == 3:
                toggle(0,1)
            elif turnState == 4:
                toggle(0,0)
                toggle(1,0)
            turnState += 1
            turnState %= 5
            lastTime = time.time()
        elif (keys['l'] and ((time.time() - lastTime) > turnsleep)):
            if turnState == 0:
                toggle(2,0)
                toggle(3,0)
                toggle(0,1)
                toggle(1,1)
            elif turnState == 1:
                toggle(2,1)
                toggle(3,0)
            elif turnState == 2:
                toggle(3,1)
            elif turnState == 3:
                toggle(2,1)
            elif turnState == 4:
                toggle(2,0)
                toggle(3,0)
            turnState += 1
            turnState %= 5
            lastTime = time.time()
        elif (state == 'd'):
            toggle(0,0)
            toggle(2,0)
            toggle(1,0)
            toggle(3,0)
        
    
finally:
    for i in range(4):
        toggle(i,0)
