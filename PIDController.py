###
#This class represents simple PID controller
#and shouldn't be used alone

class PIDController:

    #rethink, they can be different for angle and position
    __samplingTime = 0.01

    def __init__(self, gain = []):
        self.Kp = gain[0]
        self.Kd = gain[1]
        self.Ki = gain[2]
        self.integratorSum = 0
        self.lastInputValue = 0 #value inputed 

    def __gain(self, newInputValue):
        return newInputValue*self.Kp

    def __derivative(self, newInputValue):
        tmp = self.lastInputValue
        self.lastInputValue = newInputValue
        return (newInputValue - tmp)/self.__samplingTime

    def __integrate(self, newInputValue):
        self.integratorSum = self.integratorSum + newInputValue*self.__samplingTime
        return self.integratorSum

    def PIDNextStep(self, newInputValue):
        return self.__gain(newInputValue)           \
                + self.__derivative(newInputValue)  \
                + self.__integrate(newInputValue)
                


def saturate(value, maxValue, minValue):
    if maxValue < minValue:
        raise ValueError('Saturation baundries have been set improperly!')
    elif value > maxValue:
        value = maxValue
    elif value < minValue:
        value = minValue
    return value
        
