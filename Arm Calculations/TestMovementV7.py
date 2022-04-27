
import math

#Arm Measurements
BASE_HEIGHT = 76.5
BICEP_LENGTH = 128
ELBOW_LENGTH = 24
FOREARM_LENGTH = 124
WRIST_LENGTH = 146.6

#Upper and lower limites of motors, as well as their default orientations(MID)
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

#Arrays of new angles needed for pulling out/pushing in battery.
finalBicep_P = 0
finalForearm_P = 0
thetaWristND_P = 0
finalWrist_P = 0 

#Arm offset angles for given motor positions
BASE_OFFSET = math.pi/2
#BICEP_OFFSET to reset bicep to and from having a 90 degrees front
BICEP_OFFSET = math.pi/2 
#FOREARM_OFFSET used in forearm calculation, to reset forearm from having 90 degrees front
FOREARM_OFFSET = math.pi/4
WRIST_OFFSET = math.pi/2
ELBOW_OFFSET = math.pi/2

#Motor speed ratios
bicep_TO_forearm_Speed = 1
forearm_TO_wrist_Speed = 1

#Various Increments
TOLERANCE = 1 #this is how close it will try and get in mm to the target location
R_INCREMENT = math.radians(0.1) #this controls by how much it will try to increment the degrees
MAXSPEED = 40 #max speed that we want motors to turn, used for rounding speeds to nearest integer

#Receiver Lengths
GROUND_RECEIVER_LENGTH = 89 # length needed to pull/push battery free of ground receiver
GROUND_RECEIVER_CLEAR_LENGTH = 10 # desired length for battery to clear in front of ground receiver
DRONE_RECEIVER_LENGTH = 142 # length needed to pull/push battery free of drone receiver
DRONE_RECEIVER_CLEAR_LENGTH = 10 # length needed to pull/push battery to clear in front of drone receiver


#This method returns the calculated forearm theta based on measurements of the arm
# It takes the arguements of the ideal z and BicepTheta
# It returns an array of the radian measurement and the degree measurement in that order
def calcForearmTheta(P, Z,thetaBicep):
    # Zf is verticle height of the forearm
    Zf = Z-BASE_HEIGHT-BICEP_LENGTH*math.sin(thetaBicep-BICEP_OFFSET)
    if FOREARM_LENGTH >= Zf:     
        # Z_offset is to orienate the forearm to move from its 90 degree horizontal plane
        Z_OFFSET = math.radians(FOREARM_L_MID) - thetaBicep + 2*BICEP_OFFSET
        thetaForearmR=math.asin(Zf/FOREARM_LENGTH) + Z_OFFSET
        thetaForearmD=math.degrees(thetaForearmR)
        phiForearmR=math.asin(Zf/FOREARM_LENGTH)

    return ([thetaForearmR,thetaForearmD, phiForearmR])


#This method returns the calculated wrist theta based on the desired wrist orientation and the sum of all thetas
# It takes the arguements of the BicepTheta, ForearmTheta, and Wrist_Orientation_Rads
# It returns an array of the radian measurement and the degree measurement in that order
def calcWristTheta(thetaBicep, thetaForearm, orientation):
    sumAngles = BICEP_L_MID - thetaBicep + FOREARM_L_MID - thetaForearm + 90
    thetaWristD= sumAngles + orientation# - 90
    thetaWristR=math.radians(thetaWristD)

    return ([thetaWristR,thetaWristD])


#this method returns P and Z
# where P is the 'roe' the projection of the distance on the x-y plane
# Z is the height above the x-y plane
def getPZ(Zold, ThetaBi, ThetaFo):
    P = FOREARM_LENGTH*math.cos(ThetaFo) + BICEP_LENGTH*math.cos(ThetaBi-BICEP_OFFSET)
    Z = FOREARM_LENGTH*math.sin(ThetaFo) + BICEP_LENGTH*math.sin(ThetaBi-BICEP_OFFSET)  + BASE_HEIGHT

    return([P, Z])


##this method returns P and Z
# where P is the 'roe' the projection of the distance on the x-y plane after conpensating for the elbow
# Z is the height above the x-y plane after conpensating for the elbow
def elbowCorrectionForPZ(rho, z, ThetaBiN):
    adjustedThetaBiN =ThetaBiN-math.radians(90)
    #rho will never be moved further from its origin by the elbow given limits of the bicep
    #to get closer is to decrease the distance
    #Z can either be moved closer or further from its origin by the elbow given the limits of the bicep
    Z = z + ELBOW_LENGTH*math.sin(ThetaBiN)  
    P = rho + ELBOW_LENGTH*math.cos(ThetaBiN)

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
    thetaA = findATriangleAngle(aside,bside,cside)
    thetaRef = math.atan(Ztemp/Ptemp)

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


#this method readjusts the desired Pideal and Z for after the wrist orientation
# It takes the arguments of pideal, Zn, theta as Pideal, Z, and Wrist_Orientation_Rads
# It returns the adjusted pideal and Z
def adjustRhoZ(P,Z,theta):
    P = P - WRIST_LENGTH*math.sin(theta)
    Z = Z + WRIST_LENGTH*math.cos(theta)
    return ([P,Z])


#this method calculates the speeds of the motors for in synch movement when pulling/pushing
# It takes the arguments of the forearm, bicep, and wrist both before and after push/pull motions
# It returns the calculated speed ratio between the bicep/forearm or the forearm/wrist
def calcSpeeds(A1,A2,B1,B2):
    speedRatio = abs((A1-A2)/(B1-B2))
    return speedRatio


# this method intakes the calculated speeds for the motors for push/pulls
# It discover the best multiple to multiply these speeds in order to get usable integers while keeping the ratios of the speeds as true as possible
# the calcSpeeds method will output non integers, however the motors can only move at integer speeds.
# this method loops until an integer calculated speed exceeds our MAXSPEED, set at the beginnging of this code
# it then selects the best possible option from all selections of integer speeds
def calcSpeedIntegers(speed1,speed2):
    counter = 1
    best_rounded_1 = 1000
    best_rounded_2 = 1000    
    best_percent_1 = 1000
    best_percent_2 = 1000

    while(counter*speed1 < MAXSPEED and counter*speed2 < MAXSPEED):
         
        percent_floor_1 = abs(math.floor(counter*speed1) - counter*speed1)/(counter*speed1)
        percent_ceiling_1 = abs(math.ceil(counter*speed1) - counter*speed1)/(counter*speed1)
        current_percent_1 = percent_floor_1
        rounded_current_1 = math.floor(counter*speed1)
        if percent_ceiling_1 < percent_floor_1 :
            current_percent_1 = percent_ceiling_1
            rounded_current_1 = math.ceil(counter*speed1)

        percent_floor_2 = abs(math.floor(counter*speed2) - counter*speed2)/(counter*speed2)
        percent_ceiling_2 = abs(math.ceil(counter*speed2) - counter*speed2)/(counter*speed2)
        current_percent_2 = percent_floor_2
        rounded_current_2 = math.floor(counter*speed2)
        if percent_ceiling_2 < percent_floor_2 :
            current_percent_2 = percent_ceiling_2
            rounded_current_2 = math.ceil(counter*speed2)


        if current_percent_1 < best_percent_1 and current_percent_2 < best_percent_2:
            best_rounded_1 = rounded_current_1
            best_rounded_2 = rounded_current_2

        elif (current_percent_1 + current_percent_2)/2 < (best_percent_1 + best_percent_2)/2:
            best_rounded_1 = rounded_current_1
            best_rounded_2 = rounded_current_2 

        counter = counter +1

    return ([best_rounded_1, best_rounded_2])


###############################################   USER INPUT DATA BELOW    #######################################
# XYZ input below
PICKUP = {"X":-127,      
        "Y":-127,       
        "Z":145.74}      

#Desired wrist orientation in degrees, able to select anything from 0 to 180 degrees
#90 degrees is straight parallel to ground
#180 degrees is pointing straight up
#0 degrees is pointing straight down
Wrist_Orientation_D = 45
Wrist_Orientation_Rads = math.radians(Wrist_Orientation_D)

# mode controls if arm is going to pull battery out or in, or just a simple movement
# mode = 1: pulling out of ground receiver
# mode = 2: pushing into ground receiver
# mode = 3: pulling out of drone receiver
# mode = 4: pushing into drone receiver
MODE = 3
###############################################   USER INPUT DATA ABOVE    #######################################





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

    NewRhoZ = adjustRhoZ(pideal,Zn,Wrist_Orientation_Rads)
    pideal = NewRhoZ[0]
    Zn = NewRhoZ[1]

    #print("For the orientation of the wrist:  the new rho is " + str([pideal]) + "   the new Z is " + str([Zn]))

    #calculate the theta for the base
    thetaBaseN = calcBaseTheta(Xn,Yn)
    P_counter = 0
    countcycles = 1

    #CLEARANCE is the total distance needed to push/pull battery out of or fully into battery receiver
    #PERCENTAGE is the percent of CLEARANCE that the first pull/push motion will cover 
    #       FOR EXAMPLE, PERCENT = 0.7 means that the first of the two pull motions of MODE 1 will pull the battery 70% out of the receiver
    #       and the second pull will pull it out the remaining 30%
    #PERCENTAGE should be redetermined for if parameters ever change. 
    if MODE == 1 or MODE == 2:
        CLEARANCE =  GROUND_RECEIVER_LENGTH + GROUND_RECEIVER_CLEAR_LENGTH
        if MODE == 1:
            PERCENTAGE = 0.7
        elif MODE == 2:
            PERCENTAGE = 0.3
    elif MODE == 3 or MODE == 4:
        CLEARANCE =  DRONE_RECEIVER_LENGTH + DRONE_RECEIVER_CLEAR_LENGTH
        if MODE == 3:
            PERCENTAGE = 0.7
        elif MODE == 4:
            PERCENTAGE = 0.3



    #below while loop will loop three times, once for initial motor angles, then for a first set of push/pull, and then a second set of push/pull
    # multiple sets of pull/pull angles are needed as motor speeds change during the entire push/pull motion
    while (P_counter <= CLEARANCE):
#
        # below if statement will calculate an entire set of angles for a given XYZ
        if BASE_L_DOWN <= thetaBaseN[1] <= BASE_L_UP :
            #guessBicepFromTriangles is an optimization variable that helps reduce the number of cycles.
            #thetaBicepN=guessBicepFromTriangles(Xn,Yn,Zn)
            thetaBicepN=math.radians(BICEP_L_MID)

            #print("Bicep angle guess based on triangles (rad and degrees) " + str([thetaBicepN]) + str([math.degrees(thetaBicepN)]))
            updatedPZ = elbowCorrectionForPZ(pideal, Zn, thetaBicepN)

            #print("Updated P and Z are:    rho is " + str([updatedPZ[0]]) + "     Z is " + str([updatedPZ[1]]))
            thetaForearmN = calcForearmTheta(updatedPZ[0],updatedPZ[1],thetaBicepN)   #  reworked

            tempPZ = getPZ(Zn, thetaBicepN,thetaForearmN[2])
            
            #print("Temp  P and Z are:    rho is " + str([tempPZ[0]]) + "     Z is " + str([tempPZ[1]]))

        
            # below while loop will begin the guess and check method
            while not(updatedPZ[0]-TOLERANCE)<tempPZ[0]<(updatedPZ[0]+TOLERANCE):
                if tempPZ[0]>(updatedPZ[0]+1):
                    thetaBicepN = thetaBicepN+R_INCREMENT
                else:
                    thetaBicepN = thetaBicepN-R_INCREMENT
                
                updatedPZ = elbowCorrectionForPZ(pideal, Zn, thetaBicepN)
                #print("Updated P and Z are:    rho is " + str([updatedPZ[0]]) + "     Z is " + str([updatedPZ[1]]))
                thetaForearmN = calcForearmTheta(updatedPZ[0],updatedPZ[1],thetaBicepN) 
                tempPZ = getPZ(Zn, thetaBicepN,thetaForearmN[2])
                #print("NEWtemp P and Z are:    rho is " + str([tempPZ[0]]) + "     Z is " + str([tempPZ[1]]))
                countcycles += 1


            # below will assign the second set of motor angles for movement for the push/pull motion
            if P_counter == CLEARANCE:
                finalBicep_P = math.degrees(thetaBicepN)
                finalForearm_P = (thetaForearmN[1])
                thetaWristND_P = calcWristTheta(finalBicep_P,finalForearm_P,Wrist_Orientation_D)[1]
                finalWrist_P = thetaWristND_P
                unrounded_final_bicep_TO_forearm_Speed = calcSpeeds(firstBicep_P,finalBicep_P,firstForearm_P,finalForearm_P)
                unrounded_final_forearm_TO_wrist_Speed = calcSpeeds(firstForearm_P,finalForearm_P,firstWrist_P,finalWrist_P)
                finalSpeeds = calcSpeedIntegers(unrounded_final_bicep_TO_forearm_Speed,unrounded_final_forearm_TO_wrist_Speed)
                P_counter +=1

            # below will assign the first set of motor angles for movement for the push/pull motion
            elif P_counter == PERCENTAGE*CLEARANCE:
                firstBicep_P = math.degrees(thetaBicepN)
                firstForearm_P = (thetaForearmN[1])
                thetaWristND_P = calcWristTheta(firstBicep_P,firstForearm_P,Wrist_Orientation_D)[1]
                firstWrist_P = thetaWristND_P
                unrounded_first_bicep_TO_forearm_Speed = calcSpeeds(firstBicep_P,finalBicep,firstForearm_P,finalForearm)
                unrounded_first_forearm_TO_wrist_Speed = calcSpeeds(firstForearm_P,finalForearm,firstWrist_P,finalWrist)
                initialSpeeds = calcSpeedIntegers(unrounded_first_bicep_TO_forearm_Speed,unrounded_first_forearm_TO_wrist_Speed)
                P_counter = CLEARANCE
                #PP_Distance is the distance that a push/pull motion will either push or pull by
                PP_Distance = (1-PERCENTAGE)*CLEARANCE


            # below will assign the intitial set of motor angles for movement before push/pull occurs
            elif P_counter == 0:
                finalBicep = math.degrees(thetaBicepN)
                finalForearm = (thetaForearmN[1])
                thetaWristND = calcWristTheta(finalBicep,finalForearm,Wrist_Orientation_D)[1]
                finalWrist = thetaWristND 
                P_counter = PERCENTAGE*CLEARANCE
                #PP_Distance is the distance that a push/pull motion will either push or pull by
                PP_Distance = PERCENTAGE*CLEARANCE


            # Below calculates the neccessary change in XYZ for the push/pull motion
            if MODE == 1 or MODE == 3:
                pideal = pideal - (PP_Distance)*math.sin(Wrist_Orientation_Rads)
                Zn = Zn + (PP_Distance)*math.cos(Wrist_Orientation_Rads)
            elif MODE == 2 or MODE == 4:
                pideal = pideal + (PP_Distance)*math.sin(Wrist_Orientation_Rads)
                Zn = Zn - (PP_Distance)*math.cos(Wrist_Orientation_Rads)






    print('BASE ANGLE = ' + str(thetaBaseN[1]))
    print('BICEP ANGLE = ' + str(finalBicep))
    print('FOREARM ANGLE = ' + str(finalForearm))
    print('WRIST ANGLE = ' + str(finalWrist))
    #print('Number of Cycles For Results = ' + str(countcycles))

    print("First bicep Angle needed for pulling/pushing battery: "+str(firstBicep_P))
    print("First forearm Angle needed for pulling/pushing battery: "+str(firstForearm_P))
    print("First wrist Angle needed for pulling/pushing battery: "+str(firstWrist_P))
    print('Unrounded initial speed ratio of bicep/forearm is = ' + str(unrounded_first_bicep_TO_forearm_Speed))
    print('Unrounded initial speed ratio of forearm/wrist is = ' + str(unrounded_first_forearm_TO_wrist_Speed))
    print('First speed ratio of bicep/forearm is = ' + str(initialSpeeds[0]))
    print('First speed ratio of forearm/wrist is = ' + str(initialSpeeds[1]))


    print("Final bicep Angle needed for pulling/pushing battery: "+str(finalBicep_P))
    print("Final forearm Angle needed for pulling/pushing battery: "+str(finalForearm_P))
    print("Final wrist Angle needed for pulling/pushing battery: "+str(finalWrist_P))
    print('Unrounded final speed ratio of bicep/forearm is = ' + str(unrounded_final_bicep_TO_forearm_Speed))
    print('Unrounded final speed ratio of forearm/wrist is = ' + str(unrounded_final_forearm_TO_wrist_Speed))
    print('Final speed ratio of bicep/forearm is = ' + str(finalSpeeds[0]))
    print('Final speed ratio of forearm/wrist is = ' + str(finalSpeeds[1]))


        #if not(BICEP_L_DOWN>thetaBaseN[1]>BICEP_L_UP):
            #print("BICEP Can't Rotate That Far, Current Limits are: "+str(BICEP_L_DOWN)+"  -  "+str(BICEP_L_UP))
        #elif not(FOREARM_L_DOWN<thetaForearmN[1]<FOREARM_L_UP):
            #print("FOREARM Can't Rotate That Far, Current Limits are: "+str(FOREARM_L_DOWN)+"  -  "+str(FOREARM_L_UP))
        #elif not(WRIST_L_DOWN<thetaWristND<WRIST_L_UP):
            #print("WRIST Can't Rotate That Far, Current Limits are: "+str(WRIST_L_DOWN)+"  -  "+str(WRIST_L_UP))

    #else:
        #print('BASE ANGLE = ' + str(thetaBaseN[1]))
        #print("Base Can't Rotate That Far, Current Limits are: "+str(BASE_L_DOWN)+"  -  "+str(BASE_L_UP))


    return thetaBaseN[1], finalBicep, finalForearm, finalWrist, firstBicep_P, firstForearm_P, firstWrist_P, initialSpeeds[0], initialSpeeds[1], finalBicep_P, finalForearm_P, finalWrist_P, finalSpeeds[0], finalSpeeds[1], 



set_location(PICKUP)


