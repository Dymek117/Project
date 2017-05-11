import math
import time
import Controll
from threading import Timer

from SetSpeed import *
from ReadIMU import *

# ====================================================================
# ***              MAIN LOOP INITIALIZATION BEGIN                  ***
# ====================================================================


# ------------------    ITERATORS, TIMES MEASURE    ------------------

start_time = 0



# ------------------    INIT MEASUREMENT VECTORS     ------------------

IMU = [0, 0, 0]
PID = [0, 0, 0, 0, 0, 0]

pwm_timers = [0, 0, 0, 0, 0, 0]



# ----------------------    INITIALIZE GYRO     ----------------------

gyro_ref = start_dev(gyro_address, gyro_reg1, gyro_reg4, gyro_reg1_start, gyro_reg4_start)



# ------------------    INITIALIZE ACCELEROMETER     ------------------

acc_ref = start_dev(acc_address, acc_reg1, acc_reg4, acc_reg1_start, acc_reg4_start)



# --------------------    INITIALIZE PWM DRIVER     --------------------

pwm_ref = start_pwm_driver(pwm_address)



# -------------    INITIALIZE FILTER CLASS OBJECTS     -------------

x_axis = Filter()
y_axis = Filter()
z_axis = Filter()


# --------------------    INITIALIZE PID CONTROLL CLASS   --------------------

manualControll = False
referenceAngles = [0, 0, 0] #expected IMU output, error minimalized // delete info when ridden
referencePosition = [0, 0, 0] #expected position, when manualControll = True only referencePosition[2] is nessesary
actualPosition = [0, 0, 0]
controllObject = Controll.Controll([1,1,1], [1,1,1], [1,1,1], [1,1,1], [1,1,1], [1,1,1], manualControll)



# ====================================================================
# ***                MAIN LOOP INITIALIZATION END                  ***
# ====================================================================







# ====================================================================
# ***                      MAIN LOOP BEGIN                         ***
# ====================================================================


while (True):

    dt = (time.time() - start_time) % 60

    while (dt >= 0.005):

        start_time = time.time()



        # -----------------------    READ GYRO     -----------------------

        gyro_x_angle += gyro_x_read(gyro_ref)
        gyro_x_angle += gyro_y_read(gyro_ref)
        gyro_x_angle += gyro_z_read(gyro_ref)



        # -------------------    READ ACCELEROMETER     -------------------

        acc_x = acc_x_read(acc_ref)
        acc_y = acc_y_read(acc_ref)
        acc_z = acc_z_read(acc_ref)


        # ANGLE CALCULATION

        acc_x_angle = (math.atan2((acc_y), (acc_z)) + 3.141592653589793238546) * 57.29578

        acc_y_angle = (math.atan2((acc_x), (acc_z)) + 3.141592653589793238546) * 57.29578

        acc_z_angle = (math.atan2((acc_x), (acc_y)) + 3.141592653589793238546) * 57.29578



        # ----------------------    KALMAN FILTER     ----------------------

        IMU[0] = x_axis.kalman(acc_x_angle, gyro_x_angle, dt)
        IMU[1] = y_axis.kalman(acc_y_angle, gyro_y_angle, dt)
        IMU[2] = z_axis.kalman(acc_z_angle, gyro_z_angle, dt)



        # ----------------------    PID CONTROLLER    ---------------------- [ ALL PID FUNCTIONS BELOW ]

        PID = [22, 45, 45, 45, 45, 45]
        #PID = controllObject.run(referenceAngles, IMU, referencePosition, actualPosition) #saturation on 10%




        # -----------------    CONTROL MATRIX -> TIME    -------------------

        pwm_timers = set_time(PID)


        # ---------------------    RUN PWM SCHEDULE    ---------------------

        set_times(pwm_ref, pwm_timers)


# ====================================================================
# ***                       MAIN LOOP END                          ***
# ====================================================================
