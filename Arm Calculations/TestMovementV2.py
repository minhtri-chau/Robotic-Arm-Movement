

import math


#Arm Measurements
BASE_HEIGHT = 76.5
BICEP_LENGTH = 128
ELBOW_LENGTH = 24
FOREARM_LENGTH = 124
WRIST_LENGTH = 146.6

#Arm Min-Max Angles as per diagram
#old angles
#BASE_L_UP= 175
#BASE_L_DOWN = 5
#BICEP_L_DOWN= 1
#BICEP_L_UP = 179
#FOREARM_L_UP = 80
#FOREARM_L_DOWN = -80
#WRIST_L_UP = 89
#WRIST_L_DOWN = -115

BASE_L_UP= 270
BASE_L_DOWN = 90
BICEP_L_DOWN= 155
BICEP_L_UP = 245
FOREARM_L_UP = 220
FOREARM_L_DOWN = 100
WRIST_L_UP = 270
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
BICEP_OFFSET = 0
BICEP_DIRECTION = 1
FOREARM_OFFSET = 0
FOREARM_DIRECTION = 1
WRIST_OFFSET = 0
WRIST_DIRECTION = 1
#below are for end angle calculations, done before strings start to print
BICEP_OFFSET1 = math.pi/2
BICEP_DIRECTION1 = 1
FOREARM_OFFSET1 = 3*math.pi/4
FOREARM_DIRECTION = 1
WRIST_OFFSET1 = math.pi
WRIST_DIRECTION = 1

#Arm offset angles for given motor positions
FOREARM_INST_DIR = 90


#Various Increments
TOLERANCE = 1 #this is how close it will try and get in mm to the target location
R_INCREMENT = math.radians(0.1) #this controls by how much it will try to increment the degrees



PICKUP = {"X":130,      #these are to be determined
        "Y":-130,       #these are to be determined
        "Z":350}        #these are to be determined





#This method returns the calculated forearm theta based on measurements of the arm
# It takes the arguements of the ideal z and BicepTheta
# It returns an array of the radian measurement and the degree measurement in that order
def calcForearmTheta(Z,thetaBicep):
    thetaForearmR=-1*math.asin((Z-BASE_HEIGHT-BICEP_LENGTH*math.sin(thetaBicep-BICEP_OFFSET))/FOREARM_LENGTH)+FOREARM_OFFSET
    thetaForearmD=math.degrees(thetaForearmR)
    return ([thetaForearmR,thetaForearmD])

#This method returns the calculated wrist theta based on the modestatus and the sum of all thetas
# It takes the arguements of the BicepTheta, ForearmTheta, and modeStatus
# It returns an array of the radian measurement and the degree measurement in that order
def calcWristTheta(thetaBicep, thetaForearm, statusMode):
    sumAngles = -1*(thetaBicep-90) - (thetaForearm+45)
    if statusMode == 0:
        thetaWristD=sumAngles + 90
        #print('bicep ANGLE = ' + str(-1*(thetaBicep-90)))
        #print('forearm ANGLE = ' + str(thetaForearm+45))
    else:
        thetaWristD=sumAngles
    thetaWristR=math.radians(thetaWristD)
    #print('wrist ANGLE = ' + str(math.degrees(thetaWristR)))

    return ([thetaWristR,thetaWristD])


#this method returns P and Z
# where P is the 'roe' the projection of the distance on the x-y plane
# Z is the height above the x-y plane
def getPZ(ThetaBi, ThetaFo):
    P = FOREARM_LENGTH*math.cos(ThetaFo-FOREARM_OFFSET)+BICEP_LENGTH*math.cos(ThetaBi-BICEP_OFFSET)
    Z = BASE_HEIGHT+FOREARM_LENGTH*math.sin(-(ThetaFo-FOREARM_OFFSET))+BICEP_LENGTH*math.sin(ThetaBi+BICEP_OFFSET)
    return([P, Z])


#################################################################################
#################################################################################
#################################################################################
#################################################################################
#################################################################################
#################################################################################
##this method returns P and Z
# where P is the 'roe' the projection of the distance on the x-y plane after conpensating for the elbow
# Z is the height above the x-y plane after conpensating for the elbow
def elbowCorrectionForPZ(rho, z, ThetaBiN):
    adjustedThetaBiN =ThetaBiN-math.radians(90)
    P = rho - ELBOW_LENGTH*math.cos(adjustedThetaBiN)
    Z = z - ELBOW_LENGTH*math.sin(adjustedThetaBiN)   #the +/- here might vary, wait on blueprints to find out
    return([P, Z])




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
    return thetaA + refthetaCorrection*thetaRef



#This method returns the calculated base theta based on measurements of the arm
# It takes the arguements of X and Y
# It returns an array of the radian measurement and the degree measurement in that order
def calcBaseTheta(X,Y):
    thetaBaseR = math.atan(Y/X) + math.pi/2 + BASE_OFFSET
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




#takes coordinates and sends angles to motors
#Takes dictionary of "X", "Y", "Z"
#Returns nothing
# 0-base,1-bicep,2-forearm
def set_location(xyzdict):
    Xn = xyzdict["X"]
    Yn = xyzdict["Y"]
    Zn = xyzdict["Z"]
    print(str([Xn,Yn,Zn]) + ' XYZ before wrist adjustment')

    #Calculate the ideal projection on the x-y plane
    pideal = calcDist(Xn,Yn)

    # will implement a better setup for modestatus later, but want to find out about the setup first
    # might move this all to a seperate method
    # modeStatus of 0 for picking up (picks up from bottom, changing Zn by WRIST_LENGTH
    # modeStatus of 1 for delivering (drops off from side, changing pideal by WRIST_LENGTH
    modeStatus = 0
    
    ####this is for when wrist is pointing straight up changing Zn
    if modeStatus == 0:
        Zn = xyzdict["Z"] - WRIST_LENGTH + 15
    
    ####this is for when wrist is pointing sideways changing pideal
    if modeStatus == 1:
        pideal = calcDist(Xn,Yn) - WRIST_LENGTH

    print("For the given mode:     rho is " + str([pideal]) + "     Z is " + str([Zn]))

    #calculate the theta for the base
    thetaBaseN = calcBaseTheta(Xn,Yn)


    if BASE_L_DOWN <= thetaBaseN[1] <= BASE_L_UP :
        #guessBicepFromTriangles is an optimization variable that helps reduce the number of cycles.
        print(str([Xn,Yn,Zn]) + ' XYZ after wrist adjustment')
        #thetaBicepN=guessBicepFromTriangles(Xn,Yn,Zn)
        thetaBicepN=math.radians(53)      
        print("Bicep angle guess from triangles is (rad and degrees) " + str([thetaBicepN]) + str([math.degrees(thetaBicepN)]))
        updatedPZ = elbowCorrectionForPZ(pideal, Zn, thetaBicepN)
        print("Updated P and Z are:    rho is " + str([updatedPZ[0]]) + "     Z is " + str([updatedPZ[1]]))
        thetaForearmN=calcForearmTheta(updatedPZ[1],thetaBicepN)[0]   #  reworked
        tempPZ = getPZ(thetaBicepN,thetaForearmN)           #reworked
        print("Temp  P and Z are:    rho is " + str([tempPZ[0]]) + "     Z is " + str([tempPZ[1]]))
        countcycles = 0
        while not(updatedPZ[0]-TOLERANCE)<tempPZ[0]<(updatedPZ[0]+TOLERANCE):
            if tempPZ[0]>(updatedPZ[0]+1):
                thetaBicepN = thetaBicepN+R_INCREMENT
            else:
                thetaBicepN = thetaBicepN-R_INCREMENT
            
            updatedPZ = elbowCorrectionForPZ(pideal, Zn, thetaBicepN)
            print("Updated P and Z are:    rho is " + str([updatedPZ[0]]) + "     Z is " + str([updatedPZ[1]]))
            thetaForearmN=calcForearmTheta(updatedPZ[1],thetaBicepN)[0]
            tempPZ = getPZ(thetaBicepN,thetaForearmN)
            print("NEWtemp P and Z are:    rho is " + str([tempPZ[0]]) + "     Z is " + str([tempPZ[1]]))
            countcycles += 1

        thetaBicepND = math.degrees(thetaBicepN)
        thetaForearmND = math.degrees(thetaForearmN)
        thetaBaseND = thetaBaseN[1]
        thetaWristND = calcWristTheta(thetaBicepND,thetaForearmND,modeStatus)[1]

        finalBicep = thetaBicepND + 90
        finalForearm = thetaForearmND + 45 + 135
        finalWrist = thetaWristND + 180



        print('BASE ANGLE = ' + str(thetaBaseND))
        print('BICEP ANGLE = ' + str(finalBicep))
        print('FOREARM ANGLE = ' + str(finalForearm))
        print('WRIST ANGLE = ' + str(finalWrist))
        print('Number of Cycles For Answer = ' + str(countcycles))
        # XYZtemp = getXYZ(thetaBaseN[0], thetaBicepN, thetaForearmN)
        # print('XYZ Value:'+str(XYZtemp))

        return thetaBaseN[1], finalBicep, finalForearm, finalWrist

        if not(BICEP_L_DOWN<thetaBicepND<BICEP_L_UP):
            print("BICEP Can't Rotate That Far, Current Limits are: "+str(BICEP_L_DOWN)+"  -  "+str(BICEP_L_UP))
        elif not(FOREARM_L_DOWN<thetaForearmND<FOREARM_L_UP):
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





set_location(PICKUP)





