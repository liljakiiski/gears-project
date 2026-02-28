from buildhat import Motor
from buildhat import ColorSensor
from basehat import LineFinder
from basehat import IMUSensor
from basehat import HallSensor

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
sideUltra = UltrasonicSensor(18)

motorL = Motor('A') #Left motor
motorR = Motor('B') #Right motor

TARGETDIST = 10 #in cm, dist from the wall

kP = 0.6

SPEED = 80

def main():
    print("egg")

# Use just the P constant in PID to correct heading
# Option 1: Use gyroscope (area underneath the Gyro Z curve)
# Option 2: Have it be a certain dist from wall
def drive():
    dist = frontUltra.getDist

    error = dist - TARGETDIST
    correction = error * kP

    motorR.start(SPEED + correction)
    motorL.start(SPEED - correction)

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
    motorL.start(rMult * speed)
    motorR.start(lMult * speed)