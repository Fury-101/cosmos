#/ev/evle code for interfacing with the USB wireless gamepad
# This is non-blocking
# It separates button and joystick events (to lower the risk of missing a button event when moving a joystick)


# Libraries

import time
from adafruit_servokit import ServoKit

# Initialize ServoKit for the PWA board.
kit = ServoKit(channels=16)

print("Press CTRL+C to end the program.")


# Keep track of the state
FSM1State = 0
FSM1NextState = 0
angle = 0
channel = 0
kit.servo[channel].angle = angle
print ('angle: {0} \t channel: {1}'.format(angle,channel))


from evdev import InputDevice, categorize

import RPi.GPIO as GPIO
import time

 
# GPIO Mode (BOARD / BCM)
#GPIO.setmode(GPIO.BOARD)
 
# set GPIO Pins
GPIO_Ain1 = 17
GPIO_Ain2 = 27
GPIO_Apwm = 22
GPIO_Bin1 = 5
GPIO_Bin2 = 6
GPIO_Bpwm = 13

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

# Check if the gamepad is connected
# You need to adjust the event number if the wrong input device is read
gamepad = InputDevice('/dev/input/event0')
print(gamepad)

print("Press CTRL+C to end the program.")

SPEED = 100
lastTime = time.time()
heldDown = False
flag = False

lastTime2 = time.time()
heldDown2 = False
flag2 = False

stick = {
        "left": False,
        "right": False,
        "up": False,
        "down": False
}

# Main code
try:

        noError = True
        while noError:


            # Process the gamepad events
            # This implementation is non-blocking
            newbutton = False
            newstick  = False
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


            # If there was a gamepad event, show it
            if newbutton:
                print("Button: ",codebutton,valuebutton)
                if codebutton == 308 and valuebutton == 1:
                    heldDown = True
                if codebutton == 308 and valuebutton == 0:
                    heldDown = False
                if codebutton == 309 and valuebutton == 1:
                    heldDown2 = True
                if codebutton == 309 and valuebutton == 0:
                    heldDown2 = False
                
            #print(time.time() - lastTime)
            if heldDown and (time.time() - lastTime > 1):
                print("left servo moving all the way")
                if (flag):
                    kit.servo[0].angle = 0 
                else:
                    kit.servo[0].angle = 180    
                lastTime = time.time()
                flag = not flag
            if heldDown2 and (time.time() - lastTime2 > 1):
                print("right servo moving all the way")
                if (flag2):
                    kit.servo[1].angle = 0 
                else:
                    kit.servo[1].angle = 180    
                lastTime2 = time.time()
                flag2 = not flag2
            
            if newstick:
                if codestick == 1:
                    if valuestick == 0:
                        stick["up"] = True
                    if valuestick == 128:
                        stick["up"] = False
                        stick["down"] = False
                    if valuestick == 255:
                        stick["down"] = True
                if codestick == 0:
                    if valuestick == 0:
                        stick["left"] = True
                    if valuestick == 128:
                        stick["left"] = False
                        stick["right"] = False
                    if valuestick == 255:
                        stick["right"] = True
                
                if stick["up"]:
                    GPIO.output(GPIO_Ain1, False)
                    GPIO.output(GPIO_Ain2, True)
                    GPIO.output(GPIO_Bin1, False)
                    GPIO.output(GPIO_Bin2, True)
                    pwmA.ChangeDutyCycle(SPEED)                # duty cycle between 0 and 100
                    pwmB.ChangeDutyCycle(SPEED)                # duty cycle between 0 and 100
                    print("forward full speed")
                elif stick["down"]:
                    if stick["left"]:
                        GPIO.output(GPIO_Ain1, True)
                        GPIO.output(GPIO_Ain2, False)
                        GPIO.output(GPIO_Bin1, True)
                        GPIO.output(GPIO_Bin2, False)
                        pwmA.ChangeDutyCycle(SPEED)               # duty cycle between 0 and 100
                        pwmB.ChangeDutyCycle(0)               # duty cycle between 0 and 100
                        print ("Turn backwards left full speed")
                    elif stick["right"]:
                        GPIO.output(GPIO_Ain1, True)
                        GPIO.output(GPIO_Ain2, False)
                        GPIO.output(GPIO_Bin1, True)
                        GPIO.output(GPIO_Bin2, False)
                        pwmA.ChangeDutyCycle(0)               # duty cycle between 0 and 100
                        pwmB.ChangeDutyCycle(SPEED)               # duty cycle between 0 and 100
                        print ("Turn backwards right full speed")
                    else:
                        GPIO.output(GPIO_Ain1, True)
                        GPIO.output(GPIO_Ain2, False)
                        GPIO.output(GPIO_Bin1, True)
                        GPIO.output(GPIO_Bin2, False)
                        pwmA.ChangeDutyCycle(SPEED)               # duty cycle between 0 and 100
                        pwmB.ChangeDutyCycle(SPEED)               # duty cycle between 0 and 100
                        print ("Backwards full speed")
                else:
                   if stick["left"]:
                       GPIO.output(GPIO_Ain1, False)
                       GPIO.output(GPIO_Ain2, True)
                       GPIO.output(GPIO_Bin1, True) # backwards
                       GPIO.output(GPIO_Bin2, False)
                       pwmA.ChangeDutyCycle(SPEED)               # duty cycle between 0 and 100
                       pwmB.ChangeDutyCycle(SPEED)               # duty cycle between 0 and 100
                       print ("Turn left full speed")
                   elif stick["right"]:
                       GPIO.output(GPIO_Ain1, True) # backwards
                       GPIO.output(GPIO_Ain2, False) 
                       GPIO.output(GPIO_Bin1, False)
                       GPIO.output(GPIO_Bin2, True)
                       pwmA.ChangeDutyCycle(SPEED)               # duty cycle between 0 and 100
                       pwmB.ChangeDutyCycle(SPEED)               # duty cycle between 0 and 100
                       print("Turn right full speed")
                   else:
                       GPIO.output(GPIO_Ain1, False)
                       GPIO.output(GPIO_Ain2, True)
                       GPIO.output(GPIO_Bin1, False)
                       GPIO.output(GPIO_Bin2, True)
                       pwmA.ChangeDutyCycle(0)               # duty cycle between 0 and 100
                       pwmB.ChangeDutyCycle(0)               # duty cycle between 0 and 100
                       print ("Stop moving")

                #if codestick == 1 and valuestick == 0:
                #    GPIO.output(GPIO_Ain1, False)
                #    GPIO.output(GPIO_Ain2, True)
                #    GPIO.output(GPIO_Bin1, False)
                #    GPIO.output(GPIO_Bin2, True)
                #    pwmA.ChangeDutyCycle(SPEED)                # duty cycle between 0 and 100
                #    pwmB.ChangeDutyCycle(SPEED)                # duty cycle between 0 and 100
                #    print ("Forward full speed")
                #if valuestick == 128:
                #    GPIO.output(GPIO_Ain1, False)
                #    GPIO.output(GPIO_Ain2, True)
                #    GPIO.output(GPIO_Bin1, False)
                #    GPIO.output(GPIO_Bin2, True)
                #    pwmA.ChangeDutyCycle(0)               # duty cycle between 0 and 100
                #    pwmB.ChangeDutyCycle(0)               # duty cycle between 0 and 100
                #    print ("Stop moving")
                #if codestick == 1 and valuestick == 255:
                #    GPIO.output(GPIO_Ain1, True)
                #    GPIO.output(GPIO_Ain2, False)
                #    GPIO.output(GPIO_Bin1, True)
                #    GPIO.output(GPIO_Bin2, False)
                #    pwmA.ChangeDutyCycle(SPEED)               # duty cycle between 0 and 100
                #    pwmB.ChangeDutyCycle(SPEED)               # duty cycle between 0 and 100
                #    print ("Backwards full speed")
                #if codestick == 0 and valuestick == 0:
                #    GPIO.output(GPIO_Ain1, False)
                #    GPIO.output(GPIO_Ain2, True)
                #    GPIO.output(GPIO_Bin1, False)
                #    GPIO.output(GPIO_Bin2, True)
                #    pwmA.ChangeDutyCycle(100)               # duty cycle between 0 and 100
                #    pwmB.ChangeDutyCycle(0)               # duty cycle between 0 and 100
                #    print ("Turn left full speed")
                #if codestick == 0 and valuestick == 255:
                #    GPIO.output(GPIO_Ain1, False)
                #    GPIO.output(GPIO_Ain2, True)
                #    GPIO.output(GPIO_Bin1, False)
                #    GPIO.output(GPIO_Bin2, True)
                #    pwmA.ChangeDutyCycle(0)               # duty cycle between 0 and 100
                #    pwmB.ChangeDutyCycle(SPEED)               # duty cycle between 0 and 100
                #    print ("Turn right full speed")
                #print("Stick : ",codestick,valuestick)


# Quit the program when the user presses CTRL + C
except KeyboardInterrupt:
        gamepad.close()
        pwmA.stop()
        pwmB.stop()
        GPIO.cleanup()
      
