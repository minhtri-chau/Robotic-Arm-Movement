from TestMovementV2 import *
from motorctrl_v1 import * 



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

upperarm = [bicepA, forearmA]
basewrist = [baseA, wristA]

portInitialization('COM3', [5, 6])


dxlSetVelo([90,90], [5, 6])

angles_before = dxlPresAngle([5, 6])

motorRunWithInputs(upperarm, [5,6])

motorRunWithInputs(basewrist, [5,6])

motorRunWithInputs([0,0], [5,6])

angles_after = dxlPresAngle([5, 6])

portTermination()