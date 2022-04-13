
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
forearm_TO_bicep_Speed = 1
bicep_TO_wrist_Speed = 1

#Various Increments
TOLERANCE = 1 #this is how close it will try and get in mm to the target location
R_INCREMENT = math.radians(0.1) #this controls by how much it will try to increment the degrees
RECEIVER_LENGTH = 150 # length needed to pull/push battery free of receiver
RECEIVER_SPACE_LENGTH = 10 # desired length for battery to clear in front of receiver
DRONE_HOLDER_LENGTH = 150 # length needed to pull/push battery free of drone battery holder
DRONE_SPACE_LENGTH = 10 # length needed to pull/push battery to clear in front of drone battery holder


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
# It returns the calculated speed ratios between the forearm/bicep and the bicep/wrist
def calcSpeeds(forearmTheta,bicepTheta,wristTheta,PforearmTheta,PbicepTheta,PwristTheta):
    forearm_TO_bicep_Speed = abs((forearmTheta-PforearmTheta)/(bicepTheta-PbicepTheta))
    bicep_TO_wrist_Speed = abs((bicepTheta-PbicepTheta)/(wristTheta-PwristTheta))
    return ([forearm_TO_bicep_Speed,bicep_TO_wrist_Speed])


###############################################   USER INPUT DATA BELOW    #######################################
# XYZ input below
PICKUP = {"X":0,      
        "Y":289,       
        "Z":200}      

#Desired wrist orientation in degrees, able to select anything from 0 to 180 degrees
#90 degrees is straight parallel to ground
#180 degrees is pointing straight up
#0 degrees is pointing straight down
Wrist_Orientation_D = 90
Wrist_Orientation_Rads = math.radians(Wrist_Orientation_D)

# mode controls if arm is going to pull battery out or in, or just a simple movement
# mode = 0: simple movement (no pushing or pulling of battery)
# mode = 1: pulling out of receiver
# mode = 2: pushing into receiver
# mode = 3: pulling out of drone
# mode = 4: pushing into drone
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

    
    #below while loop will loop twice, once for initial motor angles, and once again for angles needed to complete a push/pull
    while (P_counter <= 1):

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

            # below will assign the first set of motor angles for movement before push/pull occurs
            if P_counter == 0:
                finalBicep = math.degrees(thetaBicepN)
                finalForearm = (thetaForearmN[1])
                thetaWristND = calcWristTheta(finalBicep,finalForearm,Wrist_Orientation_D)[1]
                finalWrist = thetaWristND 

            # below will assign the second set of motor angles for movement for the push/pull motion
            if P_counter != 0:
                finalBicep_P = math.degrees(thetaBicepN)
                finalForearm_P = (thetaForearmN[1])
                thetaWristND_P = calcWristTheta(finalBicep_P,finalForearm_P,Wrist_Orientation_D)[1]
                finalWrist_P = thetaWristND_P
                forearm_TO_bicep_Speed = calcSpeeds(finalForearm,finalBicep,finalWrist,finalForearm_P,finalBicep_P,finalWrist_P)[0]
                bicep_TO_wrist_Speed = calcSpeeds(finalForearm,finalBicep,finalWrist,finalForearm_P,finalBicep_P,finalWrist_P)[1]

            # Below calculates the neccessary change in XYZ for the push/pull motion
            if MODE == 1:
                pideal = pideal - (RECEIVER_LENGTH + RECEIVER_SPACE_LENGTH)*math.sin(Wrist_Orientation_Rads)
                Zn = Zn + (RECEIVER_LENGTH + RECEIVER_SPACE_LENGTH)*math.cos(Wrist_Orientation_Rads)
            elif MODE == 2:
                pideal = pideal + (RECEIVER_LENGTH + RECEIVER_SPACE_LENGTH)*math.sin(Wrist_Orientation_Rads)
                Zn = Zn - (RECEIVER_LENGTH + RECEIVER_SPACE_LENGTH)*math.cos(Wrist_Orientation_Rads)
            elif MODE == 3:
                pideal = pideal - (DRONE_HOLDER_LENGTH + DRONE_SPACE_LENGTH)*math.sin(Wrist_Orientation_Rads)
                Zn = Zn + (DRONE_HOLDER_LENGTH + DRONE_SPACE_LENGTH)*math.cos(Wrist_Orientation_Rads)
            elif MODE == 4:
                pideal = pideal + (DRONE_HOLDER_LENGTH + DRONE_SPACE_LENGTH)*math.sin(Wrist_Orientation_Rads)
                Zn = Zn - (DRONE_HOLDER_LENGTH + DRONE_SPACE_LENGTH)*math.cos(Wrist_Orientation_Rads)

            P_counter +=1



    print('BASE ANGLE = ' + str(thetaBaseN[1]))
    print('BICEP ANGLE = ' + str(finalBicep))
    print('FOREARM ANGLE = ' + str(finalForearm))
    print('WRIST ANGLE = ' + str(finalWrist))
    #print('Number of Cycles For Results = ' + str(countcycles))

    if MODE !=0:
        print("Bicep Angle needed for pulling/pushing battery: "+str(finalBicep_P))
        print("Forearm Angle needed for pulling/pushing battery: "+str(finalForearm_P))
        print("Wrist Angle needed for pulling/pushing battery: "+str(finalWrist_P))

    print('Speed ratio of forearm/bicep is = ' + str(forearm_TO_bicep_Speed))
    print('Speed ratio of bicep/wrist is = ' + str(bicep_TO_wrist_Speed))

        
        #if not(BICEP_L_DOWN>thetaBaseN[1]>BICEP_L_UP):
            #print("BICEP Can't Rotate That Far, Current Limits are: "+str(BICEP_L_DOWN)+"  -  "+str(BICEP_L_UP))
        #elif not(FOREARM_L_DOWN<thetaForearmN[1]<FOREARM_L_UP):
            #print("FOREARM Can't Rotate That Far, Current Limits are: "+str(FOREARM_L_DOWN)+"  -  "+str(FOREARM_L_UP))
        #elif not(WRIST_L_DOWN<thetaWristND<WRIST_L_UP):
            #print("WRIST Can't Rotate That Far, Current Limits are: "+str(WRIST_L_DOWN)+"  -  "+str(WRIST_L_UP))

    #else:
        #print('BASE ANGLE = ' + str(thetaBaseN[1]))
        #print("Base Can't Rotate That Far, Current Limits are: "+str(BASE_L_DOWN)+"  -  "+str(BASE_L_UP))


    return thetaBaseN[1], finalBicep, finalForearm, finalWrist, finalBicep_P, finalForearm_P, finalWrist_P, forearm_TO_bicep_Speed, bicep_TO_wrist_Speed



set_location(PICKUP)


