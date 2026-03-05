from buildhat import Motor

import time
import math

motorL = Motor('A')
motorR = Motor('B')

speed = 20
unitSize = 10 # cm

wheelDiameter = 4.7 # cm

points = [[1,1],[3,1],[0,2],[0,0]]

currentPoint = [0,0]

orientation = 0 # degrees, with zero as +x

motorL.set_default_speed(speed)
motorR.set_default_speed(speed)

try:
    for i in range(points.length):
        targetPoint = points[i]
        print('Navigate to ', targetPoint)
        relativeTarget = [targetPoint[0]-currentPoint[0], targetPoint[1]-currentPoint[1]]
        for j in range(2):
            if relativeTarget[j] > 0:
                targetOrientation = 0 + (j * 90)
            elif relativeTarget[j] < 0:
                targetOrientation = 180 + (j * 90)
            else
                targetOrientation = orientation # don't turn

            print('Rotate to ', targetOrientation)
            print('Drive forward ', relativeTarget[j])

            # turn(targetOrientation - orientation) , assumes positive is counterclockwise

            motorL.run_for_degrees(2 * relativeTarget[j] * unitSize / wheelDiameter, blocking=False)
            motorR.run_for_degrees(2 * relativeTarget[j] * unitSize / wheelDiameter)

            currentPoint[j] = targetPoint[j]

            orientation = targetOrientation                

except KeyboardInterrupt:
    motorL.stop()
    motorR.stop()





