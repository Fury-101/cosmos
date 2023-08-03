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
# Keep track of the state
FSM1State = 0
FSM1NextState = 0
angle = 0
channel = 0
kit.servo[channel].angle = angle
print ('angle: {0} \t channel: {1}'.format(angle,channel))


# Keep track of the timing
FSM1LastTime = 0

sleep = 1.5
turnsleep= 0.5

# Main program 
try:
        
    '''noError = True
    while noError:

        # Check the current time
        currentTime = time.time()

        # Update the state
        FSM1State = FSM1NextState


        # Check the state transitions for FSM 1
        # This is a Mealy FSM
        # State 0: angle of 0 on channel 0 or on the way there
        angle = 0
        channel = 0
        kit.servo[channel].angle = angle
        kit.servo[1].angle = 0
        print ('angle: {0} \t channel: {1}'.format(angle,channel))
                # If using a motor on channel 2
                # speed = 50
                # channel = 2
                # kit.continuous_servo[channel].throttle = speed
                # print ('speed: {0} \t channel: {1}'.format(speed,channel))
 
    kit.servo[0].angle = 0
    kit.servo[1].angle = 0
    kit.servo[2].angle = 120
    kit.servo[3].angle = 120
    time.sleep(3)
    kit.servo[0].angle = 120
    kit.servo[1].angle = 120
    kit.servo[2].angle = 0
    kit.servo[3].angle = 0
'''
    for i in range(4):
        toggle(i,0)
    time.sleep(1)
    '''while True:
        time.sleep(sleep)
        toggle(0,1)
        toggle(2,1)
        toggle(1,0)
        toggle(3,0)
        time.sleep(sleep)
        toggle(0,0)
        toggle(2,0)
        toggle(1,1)
        toggle(3,1)
    while True:
        time.sleep(sleep)
        toggle(0, 1)
        toggle(2, 1)
        time.sleep(sleep)
        toggle(1, 1)
        toggle(3, 1)
        time.sleep(sleep)
        toggle(0, 0)
        toggle(2, 0)
        time.sleep(sleep)
        toggle(1, 0)
        toggle(3, 0)
    '''
    state = 'n'
    while True:
        # Check the current time
        currentTime = time.time()

        # Update the state
        FSM1State = FSM1NextState

            
        # Process the gamepad events
        # This implementation is non-blocking
        newstick  = False
        try:
            for event in gamepad.read():            # Use this option (and comment out the next line) to react to the latest event only
                #event = gamepad.read_one()         # Use this option (and comment out the previous line) when you don't want to miss any event
                eventinfo = categorize(event)
                if event.type == 3:
                    newstick = True
                    codestick  = eventinfo.event.code
                    valuestick = eventinfo.event.value
        except:
            pass
        if (newstick and codestick == 1 and valuestick < 100):
            print('f')
            state = 'f'
        elif (newstick and codestick == 0 and valuestick < 100):
            print('l')
            state = 'l'
        elif (newstick and codestick == 0 and valuestick > 200):
            print('r')
            state = 'r'
        elif (newstick and codestick == 1 and valuestick > 200):
            print('d')
            state = 'd'
        if (state == 'f'):
            time.sleep(sleep)
            toggle(0,1)
            toggle(2,1)
            toggle(1,0)
            toggle(3,0)
            time.sleep(sleep)
            toggle(0,0)
            toggle(2,0)
            toggle(1,1)
            toggle(3,1)
        elif (state == 'r'):
            toggle(2,0)
            toggle(3,0)
            time.sleep(0.5)
            toggle(0,1)
            toggle(1,0)
            time.sleep(0.25)
            toggle(1,1)
            time.sleep(0.25)
            toggle(0,1)
            time.sleep(0.35)
            toggle(0,0)
            toggle(1,0)
        elif (state == 'l'):
            toggle(0,0)
            toggle(1,0)
            time.sleep(0.5)
            toggle(2,1)
            toggle(3,0)
            time.sleep(0.25)
            toggle(3,1)
            time.sleep(0.25)
            toggle(2,1)
            time.sleep(0.35)
            toggle(2,0)
            toggle(3,0)
        elif (state == 'd'):
            toggle(0,0)
            toggle(2,0)
            toggle(1,0)
            toggle(3,0)
    '''state = 'n'
    while True:
        # Check the current time
        currentTime = time.time()

        # Update the state
        FSM1State = FSM1NextState

            
        # Process the gamepad events
        # This implementation is non-blocking
        newstick  = False
        try:
            for event in gamepad.read():            # Use this option (and comment out the next line) to react to the latest event only
                #event = gamepad.read_one()         # Use this option (and comment out the previous line) when you don't want to miss any event
                eventinfo = categorize(event)
                if event.type == 3:
                    newstick = True
                    codestick  = eventinfo.event.code
                    valuestick = eventinfo.event.value
        except:
            pass
        if (newstick and codestick == 1 and valuestick < 100):
            print('f')
            state = 'f'
        elif (newstick and codestick == 0 and valuestick < 100):
            print('l')
            state = 'l'
        elif (newstick and codestick == 0 and valuestick > 200):
            print('r')
            state = 'r'
        if (state == 'f'):
            time.sleep(sleep)
            toggle(0, 1)
            toggle(2, 1)
            time.sleep(sleep)
            toggle(1, 1)
            toggle(3, 1)
            time.sleep(sleep)
            toggle(0, 0)
            toggle(2, 0)
            time.sleep(sleep)
            toggle(1, 0)
            toggle(3, 0)
        elif (state == 'r'):
            toggle(0, 1)
            toggle(1, 1)
            time.sleep(sleep)
            toggle(0, 0)
            toggle(1, 0)
            time.sleep(sleep)
        elif (state == 'l'):
            toggle(2, 1)
            toggle(3, 1)
            time.sleep(sleep)
            toggle(2, 0)
            toggle(3, 0)
            time.sleep(sleep)'''
        
    
finally:
    for i in range(4):
        toggle(i,0)