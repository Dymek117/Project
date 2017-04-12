import time
import math
import wiringpi2
from threading import Timer


# ====================================================================
# ***           CONFIG VARIABLES INITIALIZATION BEGIN              ***
# ====================================================================


# --------------------    I2C HARDWARE ADDRESS    --------------------

gyro_address = 0x6b
acc_address = 0x1d



# ----------------------    CONFIG REGISTERS    ----------------------

gyro_reg1 = 0b00100000
gyro_reg1_start = 0b00001111

gyro_reg4 = 0b00100011
gyro_reg4_start = 0b00110000


acc_reg1 = 0b00100000
acc_reg1_start = 0b01010111

acc_reg4 = 0b00100011
acc_reg4_start = 0b00101000



# ---------------------     WORKING VARIABLES     ---------------------

gyro_x_angle = 0
gyro_y_angle = 0
gyro_z_angle = 0

loop_time = 0.01
gyro_sens = 0.0175
dt = 0


# -------------------     MEASUREMENT VARIABLES     -------------------

cx=[0]*51
cx2=0
cx3=0

cy=[0]*51
cy2=0
cy3=0

cz=[0]*51
cz2=0
cz3=0

acx=[0]*51
acx2=0
acx3=0

acy=[0]*51
acy2=0
acy3=0

acz=[0]*51
acz2=0
acz3=0


calibrate_x = 0
calibrate_y = 0
calibrate_z = 0

acc_calibrate_x = 0
acc_calibrate_y = 0
acc_calibrate_z = 0

x_angle = 0
y_angle = 0
z_angle = 0

gyro_x_angle = 0
gyro_y_angle = 0
gyro_z_angle = 0


# --------------------    FILTER TUNING     --------------------

Q_angle = 0.001
Q_bias = 0.003
R_measure = 0.03


# ====================================================================
# ***            CONFIG VARIABLES INITIALIZATION END               ***
# ====================================================================





# ====================================================================
# ***                  IMU READ FUNCTIONS BEGIN                    ***
# ====================================================================


# =========================  I2C FUNCTIONS  ==========================


# ----------------------    INITIALIZE DEV     -----------------------

def start_dev (address, adres_rejestru1, adres_rejestru2, bajt_danych1, bajt_danych2):

    fd = wiringpi2.wiringPiI2CSetup(address)
    time.sleep(0.01)

    wiringpi2.wiringPiI2CWriteReg8(fd, adres_rejestru1, bajt_danych1)
    wiringpi2.wiringPiI2CWriteReg8(fd, adres_rejestru2, bajt_danych2)
    
    return fd



# =====================  CALIBRATION PROCEDURE  =======================

# ----------------------    GYRO CALIBRATION     ----------------------

def calibrate():

    for v in range(0,51):

      LSBx = wiringpi2.wiringPiI2CReadReg16(id, 0x22)
      MSBx = wiringpi2.wiringPiI2CReadReg16(id, 0x23)
      if MSBx & 0x8000:
         MSBx = -(0x010000 - MSBx)
      x = ((MSBx << 8) | LSBx)


      LSBy = wiringpi2.wiringPiI2CReadReg16(id, 0x24)
      MSBy = wiringpi2.wiringPiI2CReadReg16(id, 0x25)
      if MSBy & 0x8000:
          MSBy = -(0x010000 - MSBy)
      y = ((MSBy << 8) | LSBy)

    
      LSBz = wiringpi2.wiringPiI2CReadReg16(id, 0x26)
      MSBz = wiringpi2.wiringPiI2CReadReg16(id, 0x27)
      if MSBz & 0x8000:
         MSBz = -(0x010000 - MSBz)
      z = ((MSBz << 8) | LSBz)

    
      gyro_rate_x = x * gyro_sens
      cx[v]=gyro_rate_x
      global cx2
      cx2 += cx[v]

      gyro_rate_y = y * gyro_sens
      cy[v]=gyro_rate_y
      global cy2
      cy2 += cy[v]

      gyro_rate_z = z * gyro_sens
      cz[v]=gyro_rate_z
      global cz2
      cz2 += cz[v]

    calibrate_x = cx2 / 51
    calibrate_y = cy2 / 51
    calibrate_z = cz2 / 51


# ------------------    ACCELEROMETER CALIBRATION     ------------------    

def acc_calibrate(acc_id):

    for t in range(0,51):

      LSBx = wiringpi2.wiringPiI2CReadReg16(acc_id, 0x28)
      MSBx = wiringpi2.wiringPiI2CReadReg16(acc_id, 0x29)
      acc_x = ((MSBx << 8) | LSBx) >> 4

      LSBy = wiringpi2.wiringPiI2CReadReg16(acc_id, 0x2A)
      MSBy = wiringpi2.wiringPiI2CReadReg16(acc_id, 0x2B)
      acc_y = ((MSBy << 8) | LSBy) >> 4

      LSBz = wiringpi2.wiringPiI2CReadReg16(acc_id, 0x2C)
      MSBz = wiringpi2.wiringPiI2CReadReg16(acc_id, 0x2D)
      acc_z = ((MSBz << 8) | LSBz) >> 4

    
      acc_x_angle = (math.atan2(acc_y,acc_z)+3.141592653589793238546)*57.29578
      acx[t]=acc_x_angle
      global acx2
      acx2 += acx[t]

      acc_y_angle = (math.atan2(acc_x,acc_z)+3.141592653589793238546)*57.29578
      acy[t]=acc_y_angle
      global acy2
      acy2 += acy[t]

      acc_z_angle = (math.atan2(acc_x,acc_y)+3.141592653589793238546)*57.29578
      acz[t]=acc_z_angle
      global acz2
      acz2 += acz[t]

    acc_calibrate_x = acx2 / 51
    acc_calibrate_y = acy2 / 51
    acc_calibrate_z = acz2 / 51

    
    
# =====================  KALMAN FILTER  =======================

class Filter:

    # VARIABLES DECLARATION

    P = [[0 for o in range(2)] for p in range(2)]

    staticAngle = 0
    staticRate = 0
    angleDiff = 0
    bias = 0

    S = 0
    K = [0 for o in range(2)]


    def kalman(self, angle, rate, dt):

        """
        global staticAngle
        global staticRate
        global P
        global bias
        global angleDiff
        """

        self.staticRate = rate - self.bias
        self.staticAngle += self.staticRate * dt

        # COVARIANCE MATRIX, DRIFT PREDICTION

        self.P[0][0] += dt * (dt*self.P[1][1] - self.P[0][1] - self.P[1][0] + Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += Q_bias *dt

        # BIAS ESTIMATION, KALMAN AMPLIFICATION

        self.S = self.P[0][0] + R_measure
        self.K[0] = self.P[0][0] / self.S
        self.K[1] = self.P[1][0] / self.S

        self.angleDiff = angle - self.staticAngle
        self.staticAngle += self.K[0] * self.angleDiff
        self.bias += self.K[1] * self.angleDiff

        # COVARIANCE MATRIX UPDATE

        self.P[0][0] -= self.K[0] * self.P[0][0]
        self.P[0][1] -= self.K[0] * self.P[0][1]
        self.P[1][0] -= self.K[1] * self.P[0][0]
        self.P[1][1] -= self.K[1] * self.P[0][1]

        return self.staticAngle

 

# ===================== IMU Read Functions =======================

# ------------------------    GYRO READ     ------------------------

def gyro_x_read(gyro_id):

    LSBx = wiringpi2.wiringPiI2CReadReg16(gyro_id, 0x22)
    MSBx = wiringpi2.wiringPiI2CReadReg16(gyro_id, 0x23)
    if MSBx & 0x8000:
        MSBx = -(0x010000 - MSBx)
    gyro_x = ((MSBx << 8) | LSBx)

    gyro_rate_x = gyro_x * gyro_sens
    gyro_x_time = (gyro_rate_x - calibrate_x) * dt / 50
    return gyro_x_time


def gyro_y_read(gyro_id):

    LSBy = wiringpi2.wiringPiI2CReadReg16(gyro_id, 0x24)
    MSBy = wiringpi2.wiringPiI2CReadReg16(gyro_id, 0x25)
    if MSBy & 0x8000:
        MSBy = -(0x010000 - MSBy)
    gyro_y = ((MSBy << 8) | LSBy)

    gyro_rate_y = gyro_y * gyro_sens
    gyro_y_time = (gyro_rate_y - calibrate_y) * dt / 50
    return gyro_y_time


def gyro_z_read(gyro_id):

    LSBz = wiringpi2.wiringPiI2CReadReg16(gyro_id, 0x26)
    MSBz = wiringpi2.wiringPiI2CReadReg16(gyro_id, 0x27)
    if MSBz & 0x8000:
        MSBz = -(0x010000 - MSBz)

    gyro_z = ((MSBz << 8) | LSBz)

    gyro_rate_z = gyro_z * gyro_sens
    gyro_z_time = (gyro_rate_z - calibrate_z) * dt / 50
    return gyro_z_time



# --------------------    ACCELEROMETER READ     ----------------------


def acc_x_read(acc_id):

   LSBx = wiringpi2.wiringPiI2CReadReg16(acc_id, 0x28)
   MSBx = wiringpi2.wiringPiI2CReadReg16(acc_id, 0x29)
   if MSBx & 0x8000:
      MSBx = -(0x010000 - MSBx)
   acc_x = ((MSBx << 8) | LSBx) >> 4
   return acc_x


def acc_y_read(acc_id):

    LSBy = wiringpi2.wiringPiI2CReadReg16(acc_id, 0x2A)
    MSBy = wiringpi2.wiringPiI2CReadReg16(acc_id, 0x2B)
    if MSBy & 0x8000:
        MSBy = -(0x010000 - MSBy)
    acc_y = ((MSBy << 8) | LSBy) >> 4
    return acc_y


def acc_z_read(acc_id):

    LSBz = wiringpi2.wiringPiI2CReadReg16(acc_id, 0x2C)
    MSBz = wiringpi2.wiringPiI2CReadReg16(acc_id, 0x2D)
    if MSBz & 0x8000:
        MSBz = -(0x010000 - MSBz)
    acc_z = ((MSBz << 8) | LSBz) >> 4
    return acc_z
