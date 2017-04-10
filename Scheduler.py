from threading import Timer
import SetSpeed
import ReadIMU
import math
import time

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

id = start_dev(gyro_address, gyro_reg1, gyro_reg4, gyro_reg1_start, gyro_reg4_start)



# ------------------    INITIALIZE ACCELEROMETER     ------------------

id_a = start_dev(acc_address, acc_reg1, acc_reg4, acc_reg1_start, acc_reg4_start)



# --------------------    INITIALIZE PWM DRIVER     --------------------

start_pwm_driver()



# -------------    INITIALIZE FILTER CLASS OBJECTS     -------------

x_axis = Filter()
y_axis = Filter()
z_axis = Filter()



# ====================================================================
# ***                MAIN LOOP INITIALIZATION END                  ***
# ====================================================================







# ====================================================================
# ***                      MAIN LOOP BEGIN                         ***
# ====================================================================


while (True):

    while (time.time() - start_time >= 0.005):


        start_time = time.time()


        # --------------------    ESC CYCLE START     --------------------

        all_motors_on()



        # -----------------------    READ GYRO     -----------------------

        gyro_x_read()
        gyro_y_read()
        gyro_z_read()



        # -------------------    READ ACCELEROMETER     -------------------

        acc_x = acc_x_read()
        acc_y = acc_y_read()
        acc_z = acc_z_read()


        # ANGLE CALCULATION

        acc_x_angle = (math.atan2((acc_y), (acc_z)) + 3.141592653589793238546) * 57.29578

        acc_y_angle = (math.atan2((acc_x), (acc_z)) + 3.141592653589793238546) * 57.29578

        acc_z_angle = (math.atan2((acc_x), (acc_y)) + 3.141592653589793238546) * 57.29578



        # ----------------------    KALMAN FILTER     ----------------------

        IMU[0] = x_axis.kalman(acc_x_angle, gyro_x_angle, dt)
        IMU[1] = y_axis.kalman(acc_y_angle, gyro_y_angle, dt)
        IMU[2] = z_axis.kalman(acc_z_angle, gyro_z_angle, dt)



        # ----------------------    PID CONTROLLER    ---------------------- [ ALL PID FUNCTIONS IN BELOW PART ]

        PID = [50, 50, 50, 50, 50, 50]



        # ------------------    CONTROL MATRIX -> TIME    ------------------

        pwm_timers = set_time(PID)



        # ---------------------    RUN PWM SCHEDULE    ---------------------

        scheduler_start(pwm_timers)


# ====================================================================
# ***                       MAIN LOOP END                          ***
# ====================================================================











