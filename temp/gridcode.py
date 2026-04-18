'''

Just for following the grid (given certain target grid square)

'''

from buildhat import Motor

import time
import math

motorL = Motor('A')
motorR = Motor('B')

speed = 20
unitSize = 33 # cm

wheelDiameter = 4.7 # cm

points = [[3,0], [3,2], [1,3], [0,1]]

currentPoint = [0,0]

orientation = 0 # degrees, with zero as +x

motorL.set_default_speed(speed)
motorR.set_default_speed(speed)

def turn_about_self(degrees):
    WHEEL_R = 4.77 / 2 #cm?
    BOT_R = 12.9 / 2 #cm

    # the motors overshoot slightly, since two motors are running this overshooting is multiplied twice
    # account for this overshooting by subtracting a (constant?) push value
    push_per_degree = -18 / 180

    motorDegrees = (degrees + push_per_degree * degrees) * BOT_R / WHEEL_R
    print(motorDegrees)

    motorL.run_for_degrees(motorDegrees, blocking=False)
    motorR.run_for_degrees(motorDegrees)

try:
    for i in range(len(points)):
        targetPoint = points[i]
        print('Navigate to ', targetPoint)
        relativeTarget = [targetPoint[0]-currentPoint[0], targetPoint[1]-currentPoint[1]]
        for j in range(2):
            if relativeTarget[j] > 0:
                targetOrientation = 0 + (j * 90)
            elif relativeTarget[j] < 0:
                targetOrientation = 180 + (j * 90)
            else:
                targetOrientation = orientation # don't turn

            print('Rotate to ', targetOrientation)
            print('Drive forward ', relativeTarget[j])

            turn_about_self(targetOrientation - orientation)

            motorL.run_for_degrees((180 / 3.14159) * -2 * abs(relativeTarget[j]) * unitSize / wheelDiameter, blocking=False)
            motorR.run_for_degrees((180 / 3.14159) * 2 * abs(relativeTarget[j]) * unitSize / wheelDiameter)

            currentPoint[j] = targetPoint[j]

            orientation = targetOrientation                

except KeyboardInterrupt:
    motorL.stop()
    motorR.stop()





