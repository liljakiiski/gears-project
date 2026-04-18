'''

File just for testing turning

'''

from buildhat import Motor
from buildhat import ColorSensor
from basehat import LineFinder
from basehat import IMUSensor
from basehat import HallSensor
from basehat import UltrasonicSensor

import time
import math

'''Different possibilities for robot "states":
STOPPED = Robot does not move
DRIVE_FORWARD = Robot driving forward at constant speed
TURNING_LEFT = Robot is in process of steering left (90 degree turn)
TURNING_RIGHT = Robot is in process of steering right (90 degree turn)
'''
state = "STOPPED"

# Ultrasnoic Sesnors
frontUltra = UltrasonicSensor(26)
rightUltra = UltrasonicSensor(18)
leftUltra = UltrasonicSensor(24)

motorL = Motor('A') #Left motor
motorR = Motor('B') #Right motor

TARGETDIST = 0 #in cm, dist from the wall

kP = 0.6

SPEED = 20

try:
    TRACKWIDTH = rightUltra.getDist + leftUltra.getDist
except:
    print('Bad calibration')
    TRACKWIDTH = 20

def main():
    try:
        turn_about_self(360)
        while True:
            #drive()
            #turnAtIntersection()
            time.sleep(1)
    except KeyboardInterrupt:
        motorL.stop()
        motorR.stop()
        print("\nCtrl+C detected. Exiting...")

def turn_about_self(degrees):
    WHEEL_R = 4.77 / 2 #cm?
    BOT_R = 12.9 / 2 #cm

    # the motors overshoot slightly, since two motors are running this overshooting is multiplied twice
    # account for this overshooting by subtracting a (constant?) push value
    push_per_degree = -34 / 180

    motorDegrees = (degrees + push_per_degree * degrees) * BOT_R / WHEEL_R
    print(motorDegrees)

    motorL.run_for_degrees(motorDegrees, blocking=False)
    motorR.run_for_degrees(motorDegrees)

# Use just the P constant in PID to correct heading
# Option 1: Use gyroscope (area underneath the Gyro Z curve)
# Option 2: Have it be a certain dist from wall
def drive():
    rightDist = rightUltra.getDist
    leftDist = leftUltra.getDist
    
    if rightDist is None and leftDist is not None:
        print('Right out of range')
        dist = leftDist
        error = (TRACKWIDTH / 2) - dist
    elif leftDist is None and rightDist is not None:
        print('Left out of range')
        dist = rightDist
        error = dist - (TRACKWIDTH / 2)
    elif leftDist is None and rightDist is None:
        motorR.stop()
        motorL.stop()
        return
    else:
        error = rightDist - leftDist

    correction = error * kP
    
    if(SPEED - correction < 0):
        correction = SPEED
    
    if(SPEED - correction > 100):
        correction = 100 - SPEED

    motorR.start(SPEED - correction)
    motorL.start(-SPEED - correction)
    
def turnAtIntersection():
    dist = frontUltra.getDist
    
    if dist is not None:
        if dist < TRACKWIDTH / 2:
            if leftUltra.getDist is not None:
                direction = 'R'
                print('TURNING RIGHT')
            else:
                direction = 'L'
                print('TURNING LEFT')
            while(dist is not None and dist < 100):
                turn(direction, SPEED)
                dist = frontUltra.getDist
            print('DONE TURNING')

# Stop Driving
def stop():
    motorR.Stop()
    motorL.Stop()

# Drive Forward
def forward(speed):
    motorL.start(-speed)
    motorR.start(speed)

#Turn (L or R)
def turn(direction, speed):
    #multiplier to invert in case of left v. right
    rMult = 1 if (direction == "R") else -1
    lMult = -1 * rMult

    # To turn about itself the robot needs to have left and right going in opposite directions
    motorL.start(-rMult * speed)
    motorR.start(lMult * speed)

main()
