from TestMovementV3 import *
from motorctrl_v1 import * 

import time

baseA = 180
bicepA = 180
forearmA = 135
wristA = 180

PICKUP = {"X":130,    
        "Y":-130,      
        "Z":350} 

baseA, bicepA, forearmA, wristA = set_location(PICKUP)
print("\n")
print("Base angle received: " + str(baseA))
print("Bicep angle received: " + str(bicepA))
print("Forearm angle received: " + str(forearmA))
print("Wrist angle received: " + str(wristA))

goal_angles = [180, baseA+45, bicepA, forearmA, wristA]

portInitialization('COM8', [0, 1, 2, 3, 4])


dxlSetVelo([30,30,30,30,30], [0, 1, 2, 3, 4])

angles_before = dxlPresAngle([0, 1, 2, 3, 4])

motorRunWithInputs([30, 180, 90, 233, 174],[0, 1, 2, 3, 4])

motorRunWithInputs(goal_angles,[0, 1, 2, 3, 4])

time.sleep(30)

motorRunWithInputs([30, 180, 90, 233, 174],[0, 1, 2, 3, 4])

#motorRunWithInputs([0,0], [5,6])

angles_after = dxlPresAngle([0, 1, 2, 3, 4])

portTermination()