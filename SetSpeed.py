import time
import wiringpi2
from threading import Timer

# -*- coding: utf-8 -*-


#  ===================================================================
# |                                                                   |
# |  MOTOR CONTROL SCHEDULE:                                          |
# |                                                                   |
# |   1) PID Controller vector income                                 |
# |   2) ESC times calculation                                        |
# |   3) Initialize scheduler:                                        |
# |                                                                   |
# |      1) All motors enable                                         |
# |      2) Start tasks for each motor disable                        |
# |      3) Scheduler will be called every 5ms                        |
# |      4) Task reactor starts                                       |
# |                                                                   |
#  ===================================================================




# ====================================================================
# ***           CONFIG VARIABLES INITIALIZATION BEGIN              ***
# ====================================================================


# I2C HARDWARE ADDRESS

pwm_address = 0b00111110   # A6,A5 TO GND, A4-A0 TO VCC


# CONFIG REGISTERS

mode_1_reg_address = 0x00     # MODE1 REGISTER
mode_2_reg_address = 0x01     # MODE2 REGISTER

mode_1_reg_value = 0x00       # JUST CLEAR
mode_2_reg_value = 0x00       # GROUP DIMMING


# MOTOR ENABLE REGISTERS ADDRESS

north_east_address = 0x0D    # 1st, TOP ROW,    PWM 4
north_west_address = 0x0D    # 2nd, TOP ROW,    PWM 5
west_address =       0x0C    # 3rd, TOP ROW,    PWM 3
south_west_address = 0x0C    # 1st, BOTTOM ROW, PWM 2
south_east_address = 0x0C    # 2nd, BOTTOM ROW, PWM 1
east_address =       0x0C    # 3rd, BOTTOM ROW, PWM 0


# MOTOR ALL-ENABLE REGISTER VALUES [ WRITE TO BOTH LEDOUT REGISTERS! ]

motors_on_0 = 0b01010101    # PWM0, PWM1, PWM2, PWM3
motors_on_1 = 0b00000101    # PWM4, PWM5


# MOTOR DISABLE VALUES [ USE 'AND' OPERATOR ONLY! ]

north_east_off = 0b11111100   # PWM 4, LEDOUT1
north_west_off = 0b11110011   # PWM 5, LEDOUT1
west_off =       0b00111111   # PWM 3, LEDOUT0
south_west_off = 0b11001111   # PWM 2, LEDOUT0
south_east_off = 0b11111001   # PWM 1, LEDOUT0
east_off =       0b11111100   # PWM 0, LEDOUT0


# DISABLE DIMMING

dimming_address      = 0x0A    # GRPPWM
dimming_value        = 0xFF    # 99.6% [ 0.005223s ]


# WORKING VARIABLES

timer_list = [0] * 6


# TASK POINTERS

NE_task = 0
NW_task = 0
W_task = 0
SW_task = 0
SE_task = 0
E_task = 0
scheduler = 0


# ====================================================================
# ***            CONFIG VARIABLES INITIALIZATION END               ***
# ====================================================================



# ====================================================================
# ***             MOTOR CONTROLLING FUNCTIONS BEGIN                ***
# ====================================================================


# --------------  I2C FUNCTIONS  --------------


# ESTABILISH I2C CONNECTION, DRIVER CONFIGURATION

def start_pwm_driver(pwm_addr):
    pwm_id = wiringpi2.wiringPiI2CSetup(pwm_addr)
    time.sleep(0.001)
    wiringpi2.wiringPiI2CWriteReg8(pwm_id, mode_1_reg_address, mode_1_reg_value)
    
    return pwm_id



# --------------  PWM SCHEDULER FUNCTIONS  --------------

# CONVERSION [CONTROL MATRIX] -> [ESC PWM]

def set_time(control_list):

    for a in range(0, 6):
        timer_list[a] = (control_list[a] + 100) * 0.00001

    return timer_list



# TURN ON ALL MOTORS

def all_motors_on(pwm_id):

    wiringpi2.wiringPiI2CWriteReg8(pwm_id, 0x0C, 0b01010101)
    wiringpi2.wiringPiI2CWriteReg8(pwm_id, 0x0D, 0b00000101)



# START TIMER FOR AXIS TURN OFF

def start_timer(pwm_id, address, value):

    wiringpi2.wiringPiI2CWriteReg8(pwm_id, address, value)



# TURN ON MOTORS, SET TASKS FOR EVERY AXIS ENABLE TIME, TRIGGER IN 200Hz LOOP!

def scheduler_start(pwm_id, times):

    all_motors_on(pwm_id) # LOGIC '1' FOR EVERY CHANNEL

    # SCHEDULE TASKS FOR TURNING OFF EACH CHANNEL

    NE_task = Timer(times[0], start_timer, args = (pwm_id, north_east_address, north_east_off))
    NW_task = Timer(times[1], start_timer, args = (pwm_id, north_west_address, north_west_off))
    W_task = Timer(times[2], start_timer, args = (pwm_id, west_address, west_off))
    SW_task = Timer(times[3], start_timer, args = (pwm_id, south_west_address, south_west_off))
    SE_task = Timer(times[4], start_timer, args = (pwm_id, south_east_address, south_east_off))
    E_task = Timer(times[5], start_timer, args = (pwm_id, east_address, east_off))

    scheduler = Timer(0.005, scheduler_start, args = (pwm_id, timer_list))        # REPEAT AFTER 5ms

    NE_task.start()
    NW_task.start()
    W_task.start()
    SW_task.start()
    SE_task.start()
    E_task.start()

    scheduler.start()


# STOP ALL TASKS, BREAK PWM DRIVER COMMUNICATION

def scheduler_stop():

    NE_task.cancel()
    NW_task.cancel()
    W_task.cancel()
    SW_task.cancel()
    SE_task.cancel()
    E_task.cancel()

    scheduler.cancel()

