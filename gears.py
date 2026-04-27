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
MAP = 1 # given by instructional team
UNITLENGTHFOROUTPUT = 40 # in cm, size of one square in the grid, given by instructional team
UNITS = 'cm' # unit of measurement for above variable, given by instructional team
MAPNOTES = 'None'
HAZARDNOTES = 'None'
TEAM = 11

# ---------- Variable and Device Setup ----------
frontUltra = UltrasonicSensor(5)
rightUltra = UltrasonicSensor(18)
leftUltra = UltrasonicSensor(24)

irSensor = IRSensor(6, 7)
imu = IMUSensor()

motorL = Motor('A') #Left motor
motorR = Motor('B') #Right motor
dropMotor = Motor('C') # Drop motor

# VARIABLES
oldError = 0
heading = START_HEADING # 0 is facing east, 90 is facing north, 180 is facing west, 270 is facing south
currentPosition = GridSquare(ORIGIN[0], ORIGIN[1], 5) # initialize current position as origin
trackStart = 0 # in cm, reference for beginning of a straightaway
unitsFromTrackStart = 0 # in coordinate units, distance traveled since trackStart
gridKnowledge = [] # list of GridSquare objects, represents the map of the maze as the robot knows it
beginTurnPosition = -1 # position of robot when the wall disappears

# CONSTANTS
KP = 0.1
KD = 0.005
SPEED = 20
TRACKWIDTH = 20 #in cm, distance between walls, minus width of robot
UNITSIZE = 33 #in cm, size of one square in the grid
WHEELDIAMETER = 4.7 # in cm, diameter of wheel
MAGNETCONTROL = {} # 4 default imu magnetic field values, one for each direction

# ---------- Calibration ----------

gridKnowledge.append(copy.copy(currentPosition)) # add origin to map

try:
    TRACKWIDTH = rightUltra.getDist + leftUltra.getDist
    print('Track width: ', TRACKWIDTH)
except:
    print('Bad calibration')
    


# ---------- Drive and Turn Functions for Wall Following ----------

def drive():
    global beginTurnPosition
    global oldError
    rightDist = rightUltra.getDist
    leftDist = leftUltra.getDist
    frontDist = frontUltra.getDist
    
    # print('Front: ', frontDist)
    
    if updateSourcesFieldOriented(currentPosition, heading):
        print('Source detected. Reversing...')
        distanceToReverse = (get_position() - trackStart) % UNITSIZE
        print('Distance to reverse: ', distanceToReverse)
        degreesToReverse = (distanceToReverse / (math.pi * WHEELDIAMETER)) * 360
        print('Degrees to spin: ', degreesToReverse)
        motorL.run_for_degrees(degreesToReverse, blocking=False)
        motorR.run_for_degrees(-degreesToReverse, blocking=True)
        print('Force turn')
        turnAtIntersection(needToTurn=True, countNewSquare=False)
    
    if not(beginTurnPosition == -1) and get_position() > beginTurnPosition + (UNITSIZE / 2) - 2.5:
        # turn at intersection with a valid straightaway
        print('Turn right at straight-right intersection')
        turnAtIntersection(needToTurn=True)
        motorL.set_default_speed(SPEED)
        motorR.set_default_speed(SPEED)
        motorL.run_for_degrees(-500, blocking=False)
        motorR.run_for_degrees(500, blocking=True)
        print('Back in maze') 
        beginTurnPosition = -1
            
    if (rightDist is None or rightDist > 30) and (leftDist is not None and leftDist < 40):
        # right wall gone
        error = (TRACKWIDTH / 2) - leftDist - 1
        if beginTurnPosition == -1 and (frontDist is None or frontDist > 40):
            print('Straight-right intersection detected')
            beginTurnPosition = get_position()
    elif (leftDist is None or leftDist > 30) and (rightDist is not None and rightDist < 40):
        # left wall gone
        # print('Left out of range')
        error = rightDist - (TRACKWIDTH / 2)
    elif (leftDist is None or leftDist > 40) and (rightDist is None or rightDist > 40):
        # both walls gone
        # print('Both out of range')
        if(frontDist is None or frontDist > 35):
            # exited maze, stop driving
            print('Exited maze, stopping')
            end_procedure(True)
            
        error = 0
        return
    else:
        beginTurnPosition = -1
        error = rightDist - leftDist
        
        
    # print(error)

    p_correction = error * KP
    d_correction = KD * (error - oldError) / 0.05
    
    correction = p_correction + d_correction
    
    if(SPEED - correction < 0):
        correction = SPEED
    
    if(SPEED - correction > 100):
        correction = 100 - SPEED

    motorR.start(SPEED - correction)
    motorL.start(-SPEED - correction)


def turnAtIntersection(needToTurn=False, countNewSquare=True):
    global trackStart
    global unitsFromTrackStart
    global heading
    dist = frontUltra.getDist
    leftDist = leftUltra.getDist
    rightDist = rightUltra.getDist
    
    if (dist is not None and dist < (TRACKWIDTH / 2) and dist > (TRACKWIDTH / 2) - 2) or needToTurn:
        
        print('Front value:', dist)

        if rightDist is None or rightDist > 20:
            direction = -1
            print('TURNING RIGHT')
        elif (rightDist is not None and rightDist < 20) and (leftDist is None or leftDist > 20):
            direction = 1
            print('TURNING LEFT')
        else:
            direction = 2
            print('TURNING AROUND')
        turn_about_self(90 * direction)
        print('DONE TURNING')

        if(countNewSquare):
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
    push_per_degree = -28  / 180
    
    
    if(degrees == 180):
        push_per_degree /= 1.5
    

    motorDegrees = (degrees + push_per_degree * degrees) * BOT_R / WHEEL_R
    print(motorDegrees)

    motorL.run_for_degrees(motorDegrees, blocking=False)
    motorR.run_for_degrees(motorDegrees)
    
    motorL.stop()
    motorR.stop()


# ---------- Path Mapping ----------

def check_new_square():
    #print('--')
    #print(get_position())
    #print(trackStart)
    #print(unitsFromTrackStart)
    frontDist = frontUltra.getDist
    if math.floor((get_position() - trackStart) / UNITSIZE) > unitsFromTrackStart and (frontDist is None or frontDist > UNITSIZE) and beginTurnPosition == -1:
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

def get_position():
    return ((abs(motorL.get_position() - motorR.get_position())) / 2) * (math.pi * WHEELDIAMETER) / 360

def end_procedure(finished):
    if finished:
        motorL.stop()
        motorR.stop()
        motorL.set_default_speed(SPEED)
        motorR.set_default_speed(SPEED)
        motorL.run_for_degrees(-500, blocking=False)
        motorR.run_for_degrees(500, blocking=True)
        motorL.stop()
        motorR.stop()
        dropMotor.run_for_degrees(90, blocking=True)
        motorL.run_for_degrees(-500, blocking=False)
        motorR.run_for_degrees(500, blocking=True)
        dropMotor.run_for_degrees(-90, blocking=True)
    motorL.stop()
    motorR.stop()
    gridKnowledge.append(GridSquare(currentPosition.x + int(math.cos(math.radians(heading))), currentPosition.y + int(math.sin(math.radians(heading))), 4))
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
        print('Magnet Detected')
        magSquare = getRobotOrientedLoc(robotLoc, heading, magSource)
        magSquare.typo = 3
        magSquare.value = abs(imu.getMag()[2]) 
        gridKnowledge.append(magSquare)
        return True    

    if(heatSource != 'N'):
        heatSquare = getRobotOrientedLoc(robotLoc, heading, heatSource)
        heatSquare.typo = 2
        heatSquare.value = irSensor.value2
        gridKnowledge.append(heatSquare)
        print('Infared Detected')
        return True
    
    return False

'''
Return robot oriented location
'''
def getRobotOrientedLoc(robotLoc, heading, dir_):
    newLoc = copy.copy(robotLoc)

    if (heading == 0):
        if (dir_ == 'F'):
            newLoc.x += 1
        elif (dir_ == 'L'):
            newLoc.y += 1
        elif (dir_ == 'R'):
            newLoc.y -= 1
        
    elif (heading == 90):
        if (dir_ == 'F'):
            newLoc.y += 1
        elif (dir_ == 'L'):
            newLoc.x -= 1
        elif (dir_ == 'R'):
            newLoc.x += 1
            
    elif (heading == 180):
        if (dir_ == 'F'):
            newLoc.x -= 1
        elif (dir_ == 'L'):
            newLoc.y -= 1
        elif (dir_ == 'R'):
            newLoc.y += 1

    elif (heading == 270):
        if (dir_ == 'F'):
            newLoc.y -= 1
        elif (dir_ == 'L'):
            newLoc.x += 1
        elif (dir_ == 'R'):
            newLoc.x -= 1

    return newLoc

'''
Returns ROBOT ORIENTED detection of magentic source
> "N" : none detected
> "F" : detected ahead
'''
def detectMagSource():
    global heading
    MIN = 50
    
    # control = MAGNETCONTROL[str(heading)]

    '''
    Accordint to testing
    -x : ahead : look at magnitude of x (depends on polarity)
    '''
    x_mag, y_mag, z_mag = imu.getMag()

    if (abs(z_mag) >= MIN):
        print('Z Magnetic Value:', z_mag)
        return 'F'
    
    return 'N'

'''
Returns ROBOT ORIENTED detection of heat source
> "N" : none detected
> "F" : detected ahead
'''
def detectHeatSource():
    MIN = 60
    
    # print(irSensor.value2)

    if (irSensor.value2 >= MIN):
        return 'F'

    return 'N'

# ---------- CSV Conversion ----------

def convertToCSV(gridKnowledge):
    map_ = []
    max_x = max(square.x for square in gridKnowledge if square.typo == 1)
    max_y = max(square.y for square in gridKnowledge if square.typo == 1)

    for i in range(max_y + 2):
        map_.append([])
        for j in range(max_x + 2):
            square = next((sq for sq in gridKnowledge if sq.x == j and sq.y == i), None)
            if square:
                map_[i].append(square.typo)
            else:
                map_[i].append(0)
    
    map_.reverse() # invert map_ rows so up is increasing y

    with open('team11_map.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([f'Team: {TEAM}'])
        writer.writerow([f'Map: {MAP}'])
        writer.writerow([f'Unit Length: {UNITLENGTHFOROUTPUT}'])
        writer.writerow([f'Units: {UNITS}'])
        origin0 = ORIGIN[0]
        origin1 = ORIGIN[1]
        writer.writerow([f'Origin: ({origin0}',f' {origin1})'])
        writer.writerow([f'Notes: {MAPNOTES}'])
        writer.writerows(map_)
        
        
    with open('team11_hazards.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([f'Team: {TEAM}'])
        writer.writerow([f'Map: {MAP}'])
        writer.writerow([f'Notes: {HAZARDNOTES}'])
        writer.writerow([])
        writer.writerow(['Hazard Type', 'Parameter of Interest', 'Parameter Value', 'Hazard X Coordinate (cm)', 'Hazard Y Coordinate (cm)'])
        for square in gridKnowledge:
            if(square.typo == 2): # heat source
                writer.writerow(['High Temperature Heat Source', 'Radiated Power (W)', square.value, square.x * UNITLENGTHFOROUTPUT, square.y * UNITLENGTHFOROUTPUT])
            elif(square.typo == 3):
                writer.writerow(['Electrical/Magnetic Activity Source', 'Field strength (uT)', square.value, square.x * UNITLENGTHFOROUTPUT, square.y * UNITLENGTHFOROUTPUT])
    


# ---------- Main Code Loop ----------
 
try:
    
    '''
    for i in range(4):
        MAGNETCONTROL[str((heading + (90 * i)) % 360)] = imu.getMag()[2]
        turn_about_self(90)
        time.sleep(0.25)
    
    print('Magnet Calibration:')
    print(MAGNETCONTROL)
    '''
    
    # SETUP
    trackStart = get_position()
    
    
    while True:
        drive()
        turnAtIntersection()
        check_new_square()
        time.sleep(0.05)
except KeyboardInterrupt:
    print("\nCtrl+C detected. Exiting...")
    end_procedure(False )

