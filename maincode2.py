'''

The actual main code for maze mapping and project 3 demonstration.

'''

from buildhat import Motor
from basehat import IMUSensor
from basehat import IRSensor
from basehat import UltrasonicSensor
from GridSquare import GridSquare

import time
import math
import copy
import csv

# ---------- Inputs ----------

ORIGIN = (0, 0) # starting point of robot on grid, given by instructional team
START_HEADING = 90 # in degrees
MAP = 2 # given by instructional team
UNITLENGTHFOROUPUT = 30 # in cm, size of one square in the grid, given by instructional team
UNITS = 'cm' # unit of measurement for above variable, given by instructional team
NOTES = 'None'
TEAM = 11

# ---------- Variable and Device Setup ----------
frontUltra = UltrasonicSensor(26)
rightUltra = UltrasonicSensor(18)
leftUltra = UltrasonicSensor(24)

irSensor = IRSensor(0, 1)
imu = IMUSensor()

motorL = Motor('A') #Left motor
motorR = Motor('B') #Right motor

# VARIABLES
heading = START_HEADING # 0 is facing east, 90 is facing north, 180 is facing west, 270 is facing south
currentPosition = GridSquare(ORIGIN[0], ORIGIN[1], 5) # initialize current position as origin
trackStart = 0 # in cm, reference for beginning of a straightaway
unitsFromTrackStart = 0 # in coordinate units, distance traveled since trackStart
gridKnowledge = [] # list of GridSquare objects, represents the map of the maze as the robot knows it

# CONSTANTS
KP = 0.6
SPEED = 20
TRACKWIDTH = 20 #in cm, distance between walls, minus width of robot
UNITSIZE = 30 #in cm, size of one square in the grid
WHEELDIAMETER = 4.7 # in cm, diameter of wheel

# ---------- Calibration ----------

gridKnowledge.append(currentPosition) # add origin to map

try:
    TRACKWIDTH = rightUltra.getDist + leftUltra.getDist
except:
    print('Bad calibration')

# ---------- Drive and Turn Functions for Wall Following ----------

def drive():
    rightDist = rightUltra.getDist
    leftDist = leftUltra.getDist
    frontDist = frontUltra.getDist
    
    if (rightDist is None or rightDist > 20) and (leftDist is not None and leftDist < 20):
        print('Right out of range')
        error = (TRACKWIDTH / 2) - leftDist
    elif (leftDist is None or leftDist > 20) and (rightDist is not None and rightDist < 20):
        print('Left out of range')
        error = rightDist - (TRACKWIDTH / 2)
    elif (leftDist is None or leftDist > 20) and (rightDist is None or rightDist > 20):
        print('Both out of range')
        if(frontDist is None or frontDist > 40):
            # exited maze, stop driving
            print('Exited maze, stopping')
            end_procedure()
            
        error = 0
        return
    else:
        error = rightDist - leftDist
        
        
    # print(error)

    correction = error * KP
    
    if(SPEED - correction < 0):
        correction = SPEED
    
    if(SPEED - correction > 100):
        correction = 100 - SPEED

    motorR.start(SPEED - correction)
    motorL.start(-SPEED - correction)


def turnAtIntersection():
    global trackStart
    global unitsFromTrackStart
    global heading
    dist = frontUltra.getDist
    
    if dist is not None and dist < (TRACKWIDTH / 2) + 1:

        if leftUltra.getDist is not None and leftUltra.getDist < 20:
            direction = -1
            print('TURNING RIGHT')
        else:
            direction = 1
            print('TURNING LEFT')
        turn_about_self(90 * direction)
        print('DONE TURNING')

        update_map_walls()
        trackStart = get_position()
        unitsFromTrackStart = 0
        
        heading = (heading + (90 * direction)) % 360



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

# ---------- Path Mapping ----------

def check_new_square():
    #print('--')
    #print(get_position())
    #print(trackStart)
    #print(unitsFromTrackStart)
    frontDist = frontUltra.getDist
    if math.floor((get_position() - trackStart) / UNITSIZE) > unitsFromTrackStart and (frontDist is None or frontDist > UNITSIZE):
        update_map_walls()
    
def update_map_walls():
    global unitsFromTrackStart
    print('new grid point')
    unitsFromTrackStart += 1
    currentPosition.x = currentPosition.x + int(math.cos(math.radians(heading)))
    currentPosition.y = currentPosition.y + int(math.sin(math.radians(heading)))
    currentPosition.typo = 1
    print('current position', currentPosition.x, currentPosition.y)
    gridKnowledge.append(copy.copy(currentPosition))
    

    # calls to check for obstacles
    updateSourcesFieldOriented(currentPosition, heading)

def get_position():
    return ((abs(motorL.get_position() - motorR.get_position())) / 2) * (math.pi * WHEELDIAMETER) / 360

def end_procedure():
    motorL.stop()
    motorR.stop()
    for point in gridKnowledge:
        print(point.x, point.y, point.typo)

    convertToCSV(gridKnowledge)
    exit()

# ---------- Heat / Magnetic Detection -------------

'''
Passed in:
> robotLoc : an object of GridSquare
> heading: 0/90/180/270
'''
def updateSourcesFieldOriented(robotLoc, heading):
    magSource = detectMagSource()
    heatSource = detectHeatSource()

    if(magSource != 'N'):
        x_mag, y_mag, z_mag = imu.getMag()
        magSquare = getRobotOrientedLoc(robotLoc, heading, magSource)
        gridKnowledge.append(magSquare)        

    if(heatSource != 'N'):
        heatSquare = getRobotOrientedLoc(robotLoc, heading, heatSource)
        heatSquare.type = 2
        gridKnowledge.append(heatSquare)

def getRobotOrientedLoc(robotLoc, heading, dir):
    newLoc = copy.copy(robotLoc)

    if (heading == 0):
        if (dir == 'F'):
            newLoc.x += 1
        elif (dir == 'L'):
            newLoc.y += 1
        elif (dir == 'R'):
            newLoc.y -= 1
        
    elif (heading == 90):
        if (dir == 'F'):
            newLoc.y += 1
        elif (dir == 'L'):
            newLoc.x -= 1
        elif (dir == 'R'):
            newLoc.x += 1
            
    elif (heading == 180):
        if (dir == 'F'):
            newLoc.x -= 1
        elif (dir == 'L'):
            newLoc.y -= 1
        elif (dir == 'R'):
            newLoc.y += 1

    elif (heading == 270):
        if (dir == 'F'):
            newLoc.y -= 1
        elif (dir == 'L'):
            newLoc.x += 1
        elif (dir == 'R'):
            newLoc.x -= 1

    return newLoc

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
def detectHeatSource():
    MIN = 1000

    if (irSensor.value1 > MIN and irSensor.value2 > MIN):
        return 'F'
    
    elif (irSensor.value1 > MIN):
        return 'L'
    
    elif (irSensor.value2 > MIN):
        return 'R'

    return 'N'


# ---------- CSV Conversion ----------

def convertToCSV(gridKnowledge):

    map = []
    max_x = max(square.x for square in gridKnowledge if square.typo == 1)
    max_y = max(square.y for square in gridKnowledge if square.typo == 1)

    for i in range(max_y + 2):
        map.append([])
        for j in range(max_x + 2):
            square = next((sq for sq in gridKnowledge if sq.x == j and sq.y == i), None)
            if square:
                map[i].append(square.typo)
            else:
                map[i].append(0)
    
    map.reverse() # invert map rows so up is increasing y

    with open('team11_map.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Team: ', TEAM])
        writer.writerow(['Map: ', MAP])
        writer.writerow(['Unit Length: ', UNITLENGTHFOROUPUT])
        writer.writerow(['Units: ', UNITS])
        writer.writerow(['Origin: ', ORIGIN])
        writer.writerow(['Notes: ', NOTES])
        writer.writerows(map)


# ---------- Main Code Loop ----------

try:
    # SETUP
    trackStart = get_position()
    
    while True:
        drive()
        turnAtIntersection()
        check_new_square()
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nCtrl+C detected. Exiting...")
    end_procedure()