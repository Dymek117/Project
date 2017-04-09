from threading import Timer
import SetSpeed
import ReadIMU
import math
import time

# ====================================================================
# ***              MAIN LOOP INITIALIZATION BEGIN                  ***
# ====================================================================



# ------------------    INIT MEASUREMENT VECTOR     ------------------

IMU = [0, 0, 0]

# ----------------------    INITIALIZE GYRO     ----------------------

id = start_dev(gyro_address, gyro_reg1, gyro_reg4, gyro_reg1_start, gyro_reg4_start)



# ------------------    INITIALIZE ACCELEROMETER     ------------------

id_a = start_dev(acc_address, acc_reg1, acc_reg4, acc_reg1_start, acc_reg4_start)




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



# -----------------------    CALIBRATE     -----------------------

calibrate()
acc_calibrate()



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



# ====================================================================
# ***                       MAIN LOOP END                          ***
# ====================================================================











