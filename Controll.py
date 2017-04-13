import PIDController
import math
import numpy as np

###
#This class represents counts new controll
#based on what are expected values
#if there is no vehicle movement try changing or deleting saturation
###
#Creating example class
#manualControll = True
#Create controll part PIDs[x, y , z, roll, pitch, yaw]
#C = Controll.Controll([1,1,1], [1,1,1], [1,1,1], [1,1,1], [1,1,1], [1,1,1], manualControll)
###
#Running example class
#print C.run(referenceAngles(), coordinateAndAngles.kalman(), \
#            referenceCoordinate(), coordinateGPS())

class Controll:

    __vehicleWeight = 1.8 #[kg]
    __saturationMax = 99
    __saturationMin = 0
    __angleToRadianConst = 2*3.14159265359/360
    __momentsOfInertia = [1,1,1] #Moments of inertia [x, y, z]
    __sin60 = math.sin(math.radians(60))
    __sin30 = math.sin(math.radians(30))
    __vahicleRadius = 0.5 #[m]
    __thrustFactor = 1 #change name missleding, probably maximal engine thrust but not sure
    __yawFactor = 0.01 #factor for rotation around Z axis
    __controllMatrix = [[1, -__vahicleRadius*__thrustFactor*__sin60, -__vahicleRadius*__thrustFactor*__sin30,  __yawFactor],
                        [1, -__vahicleRadius*__thrustFactor*__sin60,  __vahicleRadius*__thrustFactor*__sin30, -__yawFactor],
                        [1,                                       0,          __vahicleRadius*__thrustFactor,  __yawFactor],
                        [1,  __vahicleRadius*__thrustFactor*__sin60,  __vahicleRadius*__thrustFactor*__sin30, -__yawFactor],
                        [1,  __vahicleRadius*__thrustFactor*__sin60, -__vahicleRadius*__thrustFactor*__sin30,  __yawFactor],
                        [1,                                       0,         -__vahicleRadius*__thrustFactor, -__yawFactor]]
                                                   
    #input lists of gains for each part of PID [Kp, Kd, Ki]
    def __init__(self, xPID = [], yPID = [], zPID = [],  \
                 fiPID = [], psiPID = [], thetaPID = [], \
                 manualControll = False):
        self.__xPID           = PIDController.PIDController(xPID)
        self.__yPID           = PIDController.PIDController(yPID)
        self.__zPID           = PIDController.PIDController(zPID)
        self.__fiPID          = PIDController.PIDController(fiPID)
        self.__psiPID         = PIDController.PIDController(psiPID)
        self.__thetaPID       = PIDController.PIDController(thetaPID)
        self.__valuesPWM      = [0, 0, 0, 0, 0, 0]
        self.__manualControll = manualControll

    #input expected altitude, present altitude to count new thrust value
    #check if constant component is needed
    #check how are signed axis in accelerometer, change  ref-present if needed
    def __calculateThrust(self, referenceZ, presentZ):
        errorZ= self.__zPID.PIDNextStep(referenceZ - presentZ)
        errorZ = self.__vehicleWeight*errorZ
        return PIDController.saturate(errorZ, self.__saturationMax, self.__saturationMin)

    #bear in mind that for coutnig X torque there are Y values needed
    #input expected values for fi angle and Y position,
    #input also present fi angle and Y position,
    #check how are signed axis in accelerometer, change  ref-present if needed
    def __calculateTorqueX(self, referenceY, presentY, referenceFi, presentFi):
        if self.__manualControll:
            errorFi = self.__fiPID.PIDNextStep(referenceFi - presentFi)
            errorFi = errorFi*self.__momentsOfInertia[0]
        else:
            errorY  = self.__yPID.PIDNextStep(presentY - referenceY)
            errorY  = self.__vehicleWeight*errorY
            errorY  = PIDController.saturate(errorY, self.__saturationMax, self.__saturationMin)
            errorFi = self.__fiPID.PIDNextStep(referenceFi - presentFi + errorY)
            errorFi = errorFi*self.__momentsOfInertia[0]
        return PIDController.saturate(errorFi, self.__saturationMax, self.__saturationMin)

    #bear in mind that for coutnig Y torque there are X values needed
    #input expected values for theta angle and X position,
    #input also present theta angle and X position,
    #check how are signed axis in accelerometer, change  ref-present if needed
    def __calculateTorqueY(self, referenceX, presentX, referenceTheta, presentTheta):
        if self.__manualControll:
            errorTheta = self.__thetaPID.PIDNextStep(presentTheta - referenceTheta)
            erorTheta = errorTheta*self.__momentsOfInertia[1]
        else:
            errorX     = self.__xPID.PIDNextStep(presentX - referenceX)
            errorX     = self.__vehicleWeight*errorX
            errorX     = PIDController.saturate(errorX, self.__saturationMax, self.__saturationMin)
            errorTheta = self.__thetaPID.PIDNextStep(presentTheta - referenceTheta + errorX)
            errorTheta = errorTheta*self.__momentsOfInertia[1]
        return PIDController.saturate(errorTheta, self.__saturationMax, self.__saturationMin)
  
    #check how are signed axis in accelerometer, change  ref-present if needed   
    def __calculateTorqueZ(self, referencePsi, presentPsi):
        errorPsi = self.__psiPID.PIDNextStep(referencePsi - presentPsi)
        errorPsi = errorPsi*self.__momentsOfInertia[2]
        return PIDController.saturate(errorPsi, self.__saturationMax, self.__saturationMin)

    #calculate width of PWM controll signal for each engine return as vector [6x1]
    def __calculateNewPwmValues(self, thrust, torqueX, torqueY, torqueZ):
        self.__valuesPWM = np.dot(self.__controllMatrix, [thrust, torqueX, torqueY, torqueZ]) 
        self.__valuesPWM = PIDController.saturate(self.__valuesPWM, self.__saturationMax, self.__saturationMin)

    #run whole controll algorithm
    def run(self, referenceAngle, presentAngle, referencePosition, presentPosition):
        if self.__manualControll:            
            thrust  = self.__calculateThrust(referencePosition, presentPosition)
            torqueX = self.__calculateTorqueX(referenceAngle[0], presentAngle[0])
            torqueY = self.__calculateTorqueY(referenceAngle[1], presentAngle[1])
            torqueZ = self.__calculateTorqueZ(referenceAngle[2], presentAngle[2])
        else:
            thrust  = self.__calculateThrust(referencePosition[2], presentPosition[2])
            torqueX = self.__calculateTorqueX(referencePosition[1], presentPosition[1], referenceAngle[0], presentAngle[0])
            torqueY = self.__calculateTorqueY(referencePosition[0], presentPosition[0], referenceAngle[1], presentAngle[1])
            torqueZ = self.__calculateTorqueZ(referenceAngle[2], presentAngle[2])
        self.__calculateNewPwmValues(thrust, torqueX, torqueY, torqueZ)
        return self.__valuesPWM




############  DEPRECATED  #############

"""
    def __calculateTorqueX(self, referenceFi, presentFi):
        errorFi = self.__fiPID.PIDNextStep(referenceFi - presentFi)
        errorFi = errorFi*self.__momentsOfInertia[0]
        return PIDController.saturate(errorFi, self.__saturationMax, self.__saturationMin)
"""

"""
    def __calculateTorqueY(self, referenceTheta, presentTheta):
        errorTheta = self.__thetaPID.PIDNextStep(presentTheta - referenceTheta)
        errorTheta = errorTheta*self.__momentsOfInertia[1]
        return PIDController.saturate(errorTheta, self.__saturationMax, self.__saturationMin)
"""    

"""
    def __calculateNewPwmValues(self, thrust, torqueX, torqueY, torqueZ):
        self.__valuesPWM[0] = thrust +                                      0 +                                             \
                            self.__vahicleRadius*torqueY +                  (-self.__yawFactor*torqueZ)
        
        self.__valuesPWM[1] = thrust +                                      (-self.__vahicleRadius*self.__sin60*torqueX) +  \
                            (-self.__vahicleRadius*self.__sin30*torqueY) +  self.__yawFactor*torqueZ
        
        self.__valuesPWM[2] = thrust +                                      (-self.__vahicleRadius*self.__sin60*torqueX) +  \
                            self.__vahicleRadius*self.__sin30*torqueY +     (-self.__yawFactor*torqueZ)
        
        self.__valuesPWM[3] = thrust +                                      0 +                                             \
                            self.__vahicleRadius*torqueY +                  self.__yawFactor*torqueZ
        
        self.__valuesPWM[4] = thrust +                                      self.__vahicleRadius*self.__sin60*torqueX    +  \
                            self.__vahicleRadius*self.__sin30*torqueY +     (-self.__yawFactor*torqueZ)
        
        self.__valuesPWM[5] = thrust +                                      self.__vahicleRadius*self.__sin60*torqueX    +  \
                            (-self.__vahicleRadius*self.__sin30*torqueY) +  self.__yawFactor*torqueZ
        
        PIDController.saturate(self.__valuesPWM[0], self.__saturationMax, self.__saturationMin)
        PIDController.saturate(self.__valuesPWM[1], self.__saturationMax, self.__saturationMin)
        PIDController.saturate(self.__valuesPWM[2], self.__saturationMax, self.__saturationMin)
        PIDController.saturate(self.__valuesPWM[3], self.__saturationMax, self.__saturationMin)
        PIDController.saturate(self.__valuesPWM[4], self.__saturationMax, self.__saturationMin)
        PIDController.saturate(self.__valuesPWM[5], self.__saturationMax, self.__saturationMin)

"""

"""
    def run(self, referenceZ, presentZ, referenceAngle = [], presentAngle = []):
        thrust  = self.__calculateThrust(referenceZ, presentZ)
        torqueX = self.__calculateTorqueX(referenceAngle[0], presentAngle[0])
        torqueY = self.__calculateTorqueY(referenceAngle[1], presentAngle[1])
        torqueZ = self.__calculateTorqueZ(referenceAngle[2], presentAngle[2])
        self.__calculateNewPwmValues(thrust, torqueX, torqueY, torqueZ)
        return self.__valuesPWM
"""


        
