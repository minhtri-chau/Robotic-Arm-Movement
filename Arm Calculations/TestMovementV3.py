

import math


#Arm Measurements
BASE_HEIGHT = 76.5
BICEP_LENGTH = 128
ELBOW_LENGTH = 24
FOREARM_LENGTH = 124
WRIST_LENGTH = 146.6

#Arm Min-Max Angles as per diagram
#old angles based off orginal perspectives
#BASE_L_UP= 175
#BASE_L_DOWN = 5
#BICEP_L_DOWN= 1
#BICEP_L_UP = 179
#FOREARM_L_UP = 80
#FOREARM_L_DOWN = -80
#WRIST_L_UP = 89
#WRIST_L_DOWN = -115

BASE_L_UP= 270
BASE_L_MID= 180
BASE_L_DOWN = 90
BICEP_L_DOWN= 155
BICEP_L_MID= 180
BICEP_L_UP = 245
FOREARM_L_UP = 220
FOREARM_L_MID = 135
FOREARM_L_DOWN = 100
WRIST_L_UP = 270
WRIST_L_MID = 180
WRIST_L_DOWN = -90


#Arm Positions
thetaBaseN = 0
ThetaBicepN = 0
ThetaForearmN = 0
ThetaWristN = 0


#Arm Positions in degrees
thetaBaseND = 0
ThetaBicepND = 0
ThetaForearmND = 0
ThetaWristND = 0

#Arm offset angles for given motor positions
#DIRECTION CURRENTLY UNIMPLEMENTED
#below are for math calculations, only base seems to be used
BASE_OFFSET = math.pi/2
BASE_DIRECTION = 1
#BICEP_OFFSET to reset bicep to and from having a 90 degrees front
BICEP_OFFSET = math.pi/2 
BICEP_DIRECTION = 1
#FOREARM_OFFSET used in forearm calculation, to reset forearm from having 90 degrees front
FOREARM_OFFSET = math.pi/4
FOREARM_DIRECTION = 1
WRIST_OFFSET = math.pi/2
WRIST_DIRECTION = 1
ELBOW_OFFSET = math.pi/2
ELBOW_DIRECTION = 1

#Various Increments
TOLERANCE = 1 #this is how close it will try and get in mm to the target location
R_INCREMENT = math.radians(0.1) #this controls by how much it will try to increment the degrees








#This method returns the calculated forearm theta based on measurements of the arm
# It takes the arguements of the ideal z and BicepTheta
# It returns an array of the radian measurement and the degree measurement in that order
def calcForearmTheta(P, Z,thetaBicep):
    # Zf is verticle height of the forearm
    Zf = Z-BASE_HEIGHT-BICEP_LENGTH*math.sin(thetaBicep-BICEP_OFFSET)
    if FOREARM_LENGTH >= Zf:     
        print("Current BICEP angle:   "+ str([math.degrees(thetaBicep)]))   
        print("Current Z is:   "+ str([Z]))
        print("Remaining Z is:   "+ str([Zf]))
        # Z_offset is to orienate the forearm to move from its 90 degree horizontal plane
        Z_OFFSET = math.radians(FOREARM_L_MID) - thetaBicep + 2*BICEP_OFFSET
        thetaForearmR=math.asin(Zf/FOREARM_LENGTH) + Z_OFFSET
        thetaForearmD=math.degrees(thetaForearmR)
        phiForearmR=math.asin(Zf/FOREARM_LENGTH)
        print("Current Forearm angle:   "+ str([thetaForearmD]))
        print("Current phi  angle:   "+ str([math.degrees(phiForearmR)]))

    return ([thetaForearmR,thetaForearmD, phiForearmR])
    #else:
        #print('testing point #3:  forearm not long enough')
       # print("Forearm must be atleast:    " + str([Zf]))
        #return ([0,0])


#This method returns the calculated wrist theta based on the modestatus and the sum of all thetas
# It takes the arguements of the BicepTheta, ForearmTheta, and modeStatus
# It returns an array of the radian measurement and the degree measurement in that order
def calcWristTheta(thetaBicep, thetaForearm, statusMode):
    sumAngles = BICEP_L_MID - thetaBicep + FOREARM_L_MID - thetaForearm + 90
    if statusMode == 0:
        thetaWristD= sumAngles + WRIST_L_MID

    else:
        thetaWristD=sumAngles + WRIST_OFFSET
    thetaWristR=math.radians(thetaWristD)

    return ([thetaWristR,thetaWristD])


#this method returns P and Z
# where P is the 'roe' the projection of the distance on the x-y plane
# Z is the height above the x-y plane
def getPZ(Zold, ThetaBi, ThetaFo):
    P = FOREARM_LENGTH*math.cos(ThetaFo) + BICEP_LENGTH*math.cos(ThetaBi-BICEP_OFFSET)
    #Z = Zold + FOREARM_LENGTH*math.sin(ThetaFo) + BICEP_LENGTH*math.sin(ThetaBi+BICEP_OFFSET)
    Z = FOREARM_LENGTH*math.sin(ThetaFo) + BICEP_LENGTH*math.sin(ThetaBi-BICEP_OFFSET)  + BASE_HEIGHT

    #print('rho = ' + str(P))
    return([P, Z])


##this method returns P and Z
# where P is the 'roe' the projection of the distance on the x-y plane after conpensating for the elbow
# Z is the height above the x-y plane after conpensating for the elbow
def elbowCorrectionForPZ(rho, z, ThetaBiN):
    adjustedThetaBiN =ThetaBiN-math.radians(90)
    #rho will never be moved further from its origin by the elbow given limits of the bicep
    #to get closer is to decrease the distance
    ####P = rho + ELBOW_LENGTH*math.cos(adjustedThetaBiN)
    #Z can either be moved closer or further from its origin by the elbow given the limits of the bicep
    #if ThetaBiN > BICEP_L_MID:
    Z = z + ELBOW_LENGTH*math.sin(ThetaBiN)  
    P = rho + ELBOW_LENGTH*math.cos(ThetaBiN)
   # if ThetaBiN == 190.95:
      #  Z = z - ELBOW_LENGTH*math.sin(adjustedThetaBiN)  
       # P = rho
    #if BICEP_L_MID > ThetaBiN < 190.95:
       # Z = z - ELBOW_LENGTH*math.sin(adjustedThetaBiN)  
       # P = rho + ELBOW_LENGTH*math.cos(adjustedThetaBiN)
    #elif ThetaBiN == BICEP_L_MID:  # rho is closer
       # Z = z  
       # P = rho - ELBOW_LENGTH  ######*math.cos(adjustedThetaBiN) cos(180) = -1  
    #elif ThetaBiN < BICEP_L_MID:   # rho is closer
     #   Z = z + ELBOW_LENGTH*math.sin(ThetaBiN) 
      #  P = rho + ELBOW_LENGTH*math.cos(ThetaBiN)
        #print('rho = ' + str(ELBOW_LENGTH*math.cos(ThetaBiN)))
        #print('Bicep angle = ' + str(math.degrees(ThetaBiN)))

    return([P, Z])

###might be able to eliminate each rho statement at at one at the end!!!!!!!!!!!!!!!!!!!!!!!
######################################## the same goes for Z

### the above if statement might not be neccessary, as sin(bicep) autimatically turns from +/-
#test at end to see if it can be removed


#This method gives a close approximation for the bicep angle
# It takes the arguements of X, Y, Z
# It returns the angle of the bicep
def guessBicepFromTriangles(X,Y,Z):
    Ztemp = Z - BASE_HEIGHT
    Ptemp = calcDist(X,Y)
    refthetaCorrection = 1
    if Ztemp<0:
        refthetaCorrection = -1
        Ztemp = abs(Ztemp)
    aside = FOREARM_LENGTH
    bside = BICEP_LENGTH
    cside = calcDist(Ptemp,Ztemp)
    ##########print(str([aside,bside,cside]) + ' ABC sides')
    thetaA = findATriangleAngle(aside,bside,cside)
    thetaRef = math.atan(Ztemp/Ptemp)
    #theta = thetaA + refthetaCorrection*thetaRef + BICEP_OFFSET
    #phi = thetaA + refthetaCorrection*thetaRef
    return thetaA + refthetaCorrection*thetaRef + BICEP_OFFSET  #this pi/2 may need to be fixed, i dont recall chrises original angles



#This method returns the calculated base theta based on measurements of the arm
# It takes the arguements of X and Y
# It returns an array of the radian measurement and the degree measurement in that order
def calcBaseTheta(X,Y):
    if X == 0:
        thetaBaseR = math.pi
    #below accounts for positive x values
    else:
        thetaBaseR = math.atan(Y/X) + BASE_OFFSET
        #below accounts for negative x values
        if X < 0:
            X = abs(X)
            thetaBaseR = math.atan(Y/X) + BASE_OFFSET
            thetaBaseR = math.pi*2 - thetaBaseR
    thetaBaseD = math.degrees(thetaBaseR)
    return ([thetaBaseR,thetaBaseD])


#This method calculates the projection of distance on the X Y plane given an X and Y
# It takes the arguments of X and Y in mm
# It Returns P
def calcDist(X,Y):
    return  math.sqrt((X**2)+(Y**2))

#this method calculates the angle opposed to side A
# It takes the arguments of A,B,C the sides of the triangle in mm
# It returns the angle of the opposed to A
def findATriangleAngle(A,B,C):
    thetaA = math.acos((B**2+C**2-A**2)/(2*B*C))
    return thetaA



PICKUP = {"X":150,      #these are to be determined
        "Y":150,       #these are to be determined
        "Z":100}        #these are to be determined

#takes coordinates and sends angles to motors
#Takes dictionary of "X", "Y", "Z"
#Returns angles for the four motors
def set_location(xyzdict):
    Xn = xyzdict["X"]
    Yn = xyzdict["Y"]
    Zn = xyzdict["Z"]
    print(str([Xn,Yn,Zn]) + ' XYZ before wrist adjustment')

    #Calculate the ideal projection on the x-y plane
    pideal = calcDist(Xn,Yn)
    #Zn = Zn - BASE_HEIGHT

    # will implement a better setup for modestatus later, but want to find out about the setup first
    # might move this all to a seperate method
    # modeStatus of 0 for picking up (picks up from bottom, changing Zn by WRIST_LENGTH
    # modeStatus of 1 for picking up (picks up from above, changing Zn by WRIST_LENGTH
    # modeStatus of 2 for delivering (drops off from side, changing pideal by WRIST_LENGTH
    modeStatus = 2
    
    ####this is for when wrist is pointing straight up changing Zn
    if modeStatus == 0:
        Zn = Zn - WRIST_LENGTH
    
    ####this is for when wrist is pointing straight down changing Zn
    elif modeStatus == 1:
        Zn = Zn + WRIST_LENGTH

    ####this is for when wrist is pointing sideways changing pideal
    elif modeStatus == 2:
        pideal = pideal - WRIST_LENGTH

    print("For the given mode:  the new rho is " + str([pideal]) + "   the new Z is " + str([Zn]))

    #calculate the theta for the base
    thetaBaseN = calcBaseTheta(Xn,Yn)


    if BASE_L_DOWN <= thetaBaseN[1] <= BASE_L_UP :
        #guessBicepFromTriangles is an optimization variable that helps reduce the number of cycles.
        thetaBicepN=guessBicepFromTriangles(Xn,Yn,Zn)
        print("Bicep angle guess based on triangles (rad and degrees) " + str([thetaBicepN]) + str([math.degrees(thetaBicepN)]))
        updatedPZ = elbowCorrectionForPZ(pideal, Zn, thetaBicepN)

        print("Updated P and Z are:    rho is " + str([updatedPZ[0]]) + "     Z is " + str([updatedPZ[1]]))
        thetaForearmN = calcForearmTheta(updatedPZ[0],updatedPZ[1],thetaBicepN)   #  reworked
        print('Forearm ANGLE = ' + str(thetaForearmN[1]))

        tempPZ = getPZ(Zn, thetaBicepN,thetaForearmN[2])
        
        print("Temp  P and Z are:    rho is " + str([tempPZ[0]]) + "     Z is " + str([tempPZ[1]]))
        countcycles = 0
    

        while not(updatedPZ[0]-TOLERANCE)<tempPZ[0]<(updatedPZ[0]+TOLERANCE):
            if tempPZ[0]>(updatedPZ[0]+1):
                thetaBicepN = thetaBicepN+R_INCREMENT
            else:
                thetaBicepN = thetaBicepN-R_INCREMENT
            
            updatedPZ = elbowCorrectionForPZ(pideal, Zn, thetaBicepN)
            print("Updated P and Z are:    rho is " + str([updatedPZ[0]]) + "     Z is " + str([updatedPZ[1]]))
            thetaForearmN = calcForearmTheta(updatedPZ[0],updatedPZ[1],thetaBicepN)   #  reworked
            tempPZ = getPZ(Zn, thetaBicepN,thetaForearmN[2])
            print("NEWtemp P and Z are:    rho is " + str([tempPZ[0]]) + "     Z is " + str([tempPZ[1]]))
            countcycles += 1


        finalBicep = math.degrees(thetaBicepN)
        finalForearm = (thetaForearmN[1])
        thetaWristND = calcWristTheta(finalBicep,finalForearm,modeStatus)[1]
        finalWrist = thetaWristND 


        print('BASE ANGLE = ' + str(thetaBaseN[1]))
        print('BICEP ANGLE = ' + str(finalBicep))
        print('FOREARM ANGLE = ' + str(finalForearm))

        print('WRIST ANGLE = ' + str(finalWrist))
        print('Number of Cycles For Results = ' + str(countcycles))
        # XYZtemp = getXYZ(thetaBaseN[0], thetaBicepN, thetaForearmN)
        # print('XYZ Value:'+str(XYZtemp))

        
        if not(BICEP_L_DOWN>thetaBaseN[1]>BICEP_L_UP):
            print("BICEP Can't Rotate That Far, Current Limits are: "+str(BICEP_L_DOWN)+"  -  "+str(BICEP_L_UP))
        elif not(FOREARM_L_DOWN<thetaForearmN[1]<FOREARM_L_UP):
            print("FOREARM Can't Rotate That Far, Current Limits are: "+str(FOREARM_L_DOWN)+"  -  "+str(FOREARM_L_UP))
        elif not(WRIST_L_DOWN<thetaWristND<WRIST_L_UP):
            print("WRIST Can't Rotate That Far, Current Limits are: "+str(WRIST_L_DOWN)+"  -  "+str(WRIST_L_UP))
        #else:
            #calcSpeedDiff(thetaBicepND, thetaForearmND, thetaWristND)
            #moveresults = setAnglesCorrected([thetaBaseND,thetaBicepND,thetaForearmND,thetaWristND])
            #print('Arm Move Successful = ' + str(moveresults))
    else:
        print('BASE ANGLE = ' + str(thetaBaseN[1]))
        print("Base Can't Rotate That Far, Current Limits are: "+str(BASE_L_DOWN)+"  -  "+str(BASE_L_UP))


    return thetaBaseN[1], finalBicep, finalForearm, finalWrist



set_location(PICKUP)





