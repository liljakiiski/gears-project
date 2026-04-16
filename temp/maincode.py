'''

Main code for maze mapping and project 3 demonstration.

'''

from buildhat import Motor
from basehat import IMUSensor
from basehat import IRSensor
from basehat import UltrasonicSensor

import time
import math

# ---------- Variable and Device Setup ----------
frontUltra = UltrasonicSensor(26)
rightUltra = UltrasonicSensor(18)
leftUltra = UltrasonicSensor(24)

irSensor = IRSensor(0, 1)
imu = IMUSensor()

motorL = Motor('A') #Left motor
motorR = Motor('B') #Right motor

# cON
TARGETDIST = 0 #in cm, dist from the 

KP = 0.6
SPEED = 20

# ---------- Calibration ----------

try:
    TRACKWIDTH = rightUltra.getDist + leftUltra.getDist
except:
    print('Bad calibration')
    TRACKWIDTH = 20




# ---------- Drive and Turn Functions for Wall Following ----------

def drive():
    rightDist = rightUltra.getDist
    leftDist = leftUltra.getDist
    
    if (rightDist is None or rightDist > 20) and leftDist is not None:
        print('Right out of range')
        error = (TRACKWIDTH / 2) - leftDist
    elif (leftDist is None or leftDist > 20) and rightDist is not None:
        print('Left out of range')
        error = rightDist - (TRACKWIDTH / 2)
    elif leftDist is None and rightDist is None:
        print('Both out of range')
        error = 0
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
        if dist < (TRACKWIDTH / 2) + 1:
            if leftUltra.getDist is not None and leftUltra.getDist < 20:
                direction = -1
                print('TURNING RIGHT')
            else:
                # print('Left sensor: ' + leftUltra.getDist)
                direction = 1
                print('TURNING LEFT')
            turn_about_self(90 * direction)
            print('DONE TURNING')


def turn_about_self(degrees):
    motorL.set_default_speed(15)
    motorR.set_default_speed(15)
    
    WHEEL_R = 4.77 / 2 #cm?
    BOT_R = 12.9 / 2 #cm

    # the motors overshoot slightly, since two motors are running this overshooting is multiplied twice
    # account for this overshooting by subtracting a (constant?) push value
    push_per_degree = -34 / 180

    motorDegrees = (degrees + push_per_degree * degrees) * BOT_R / WHEEL_R
    print(motorDegrees)

    motorL.run_for_degrees(motorDegrees, blocking=False)
    motorR.run_for_degrees(motorDegrees)



# ---------- Heat / Magnetic Detection -------------

'''
Returns ROBOT ORIENTED detection of magentic source

> "N" : none detected
> "L" : detect left side
> "R" : detected right side
> "F" : detected ahead
'''
def detectMagSource():
    MIN = 1000

    '''
    Accordint to image on IMU
    -x : ahead
    -y : left
    +y : right
    '''
    x_mag, y_mag, z_mag = imu.getMag()

    if (abs(x_mag) > MIN):
        return 'F'
    
    elif (y_mag > MIN):
        return 'L'

    elif (y_mag < -MIN):
        return 'R'
    
    return 'N'

'''
Returns ROBOT ORIENTED detection of heat source

> "N" : none detected
> "L" : detect left side
> "R" : detected right side
> "F" : detected ahead
'''
def detectHeatSources():
    MIN = 1000

    if (irSensor.value1 > MIN and irSensor.value2 > MIN):
        return 'F'
    
    elif (irSensor.value1 > MIN):
        return 'L'
    
    elif (irSensor.value2 > MIN):
        return 'R'

    return 'N'

# ---------- Classes Idk ---------------------------
class GridSquare:
    ''' 
    x = coordinate x
    y = coordinate y
    type = one of the following options
    > 0 : unknown
    > 1 : traversed square
    > 2 : heat source
    > 3 : mag source
    > 4 : end
    > 5 : origin
    '''
    def __init__(x, y, type):
        self.x = x
        self.y = y
        self.type = type




# ---------- Main Code Loop ----------

try:
    while True:
        drive()
        turnAtIntersection()
        time.sleep(0.1)
except KeyboardInterrupt:
    motorL.stop()
    motorR.stop()
    print("\nCtrl+C detected. Exiting...")