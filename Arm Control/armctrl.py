from TestMovementV2 import *
import motorctrl_v1



baseA = 180
bicepA = 180
forearmA = 135
wristA = 180

PICKUP = {"X":130,    
        "Y":-130,      
        "Z":350} 

baseA, bicepA, forearmA, wristA = set_location(PICKUP)
print("\n\n")
print("Base angle received: " + str(baseA))
print("Bicep angle received: " + str(bicepA))
print("Forearm angle received: " + str(forearmA))
print("Wrist angle received: " + str(wristA))

upperarm = [bicepA, forearmA]

motorctrl_v1.portInitialization('COM3', [5, 6])


motorctrl_v1.dxlSetVelo([90,90], [5, 6])

angles_before = motorctrl_v1.dxlPresAngle([5, 6])

motorctrl_v1.motorRunWithInputs(upperarm, [5,6])

motorctrl_v1.motorRunWithInputs([0,0], [5,6])

angles_after = motorctrl_v1.dxlPresAngle([5, 6])

motorctrl_v1.portTermination()