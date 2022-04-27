from TestMovementV7 import *
from motorctrl_v1 import * 

import time

baseA = 180
bicepA = 180
forearmA = 135
wristA = 180

PICKUP1 = {"X":0,      
        "Y":320,       
        "Z":189}  



clawA_Close = 205
clawA_Op = 120

baseA, bicepA, forearmA, wristA, bicepA_P, forearmA_P, wristA_P, bispeed_P, forspeed_P, wrtspeed_P, bicepA_F, forearmA_F, wristA_F, bispeed_F, forspeed_F, wrtspeed_F = set_location(PICKUP1)
print("\n")

print("Base angle received: " + str(baseA))
print("Bicep angle received: " + str(bicepA))
print("Forearm angle received: " + str(forearmA))
print("Wrist angle received: " + str(wristA) + "\n")


print("Bicep angle for pulling received: " + str(bicepA_P))
print("Forearm angle for pulling received: " + str(forearmA_P))
print("Wrist angle for pulling received: " + str(wristA_P) + "\n")

print("Bicep speed for pulling received: " + str(bispeed_P))
print("Forearm speed for pulling received: " + str(forspeed_P))
print("Wrist speed for pulling received: " + str(wrtspeed_P) + "\n")

print("Final Bicep angle received: " + str(bicepA_F))
print("Final Forearm angle received: " + str(forearmA_F))
print("Final Wrist angle received: " + str(wristA_F) + "\n")

print("Final Bicep speed for received: " + str(bispeed_F))
print("Final Forearm speed for received: " + str(forspeed_F))
print("Final Wrist speed for received: " + str(wrtspeed_F)+ "\n")

front_angles = [clawA_Op, wristA, forearmA, bicepA, baseA+40]

pulling_angles = [bicepA_P, forearmA_P, wristA_P]

final_angles = [bicepA_F, forearmA_F, wristA_F]

pulling_speeds = [bispeed_P, forspeed_P, wrtspeed_P]

final_speeds = [bispeed_F, forspeed_F, wrtspeed_F]

up_angles = [154, 218, 215, 171, 144]

before_ground_angles = [152, 114, 206, 190, 61]

before_push_angles = [152, 115, 189, 194, 68]

push_angles =[152, 115, 196, 153, 102]

push_final_angles = [152, 114, 184, 147, 122]
#----------------------------------------------------

dxlIDs =  [0, 1, 2, 3, 4]
portInitialization('COM8', [0, 1, 2, 3, 4])
dxlSetVelo([40,40,40,40,40], [0, 1, 2, 3, 4])

dxlPresAngle(dxlIDs)

# #move arm in front of the battery
motorRunWithInputs(front_angles,[0, 4, 3, 2, 1])
time.sleep(5)

# #close the wrist
motorRunWithInputs([clawA_Close],[0])
time.sleep(3)


# # #pull the battery out
dxlSetVelo(pulling_speeds, [2, 3, 4])
simMotorRun(pulling_angles, [2, 3, 4])
time.sleep(3)

# # #final the position of the arm
dxlSetVelo(final_speeds, [2, 3, 4])
simMotorRun(final_angles, [2, 3, 4])
time.sleep(5)

dxlSetVelo([30,30,30,30,30], [0, 1, 2, 3, 4])
simMotorRun(up_angles, [0, 1, 2, 3, 4])
time.sleep(5)

simMotorRun(before_ground_angles, [0, 1, 2, 3, 4])
time.sleep(5)


portTermination()