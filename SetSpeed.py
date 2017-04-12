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
mode_1_reg_value = 0x00       # ENABLE OSCILLATOR


# MOTOR ENABLE REGISTERS ADDRESS

PWM_ON_ADDR_LOW = [0x06, 0x0A, 0x0E, 0x12, 0x16, 0x1A]
PWM_ON_ADDR_HIGH = [0x07, 0x0B, 0x0F, 0x13, 0x17, 0x1B]


# MOTOR DISABLE REGISTERS ADDRESS

PWM_OFF_ADDR_LOW = [0x08, 0x0C, 0x10, 0x14, 0x18, 0x1C]
PWM_OFF_ADDR_HIGH = [0x09, 0x0D, 0x11, 0x15, 0x19, 0x1D]


# DISABLE DIMMING

dimming_address      = 0x0A    # GRPPWM
dimming_value        = 0xFF    # 99.6% [ 0.005223s ]


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
    
    # OSCILLATOR = ON 
    wiringpi2.wiringPiI2CWriteReg8(pwm_id, mode_1_reg_address, mode_1_reg_value)

    # FREQ = 200Hz        
    wiringpi2.wiringPiI2CWriteReg8(pwm_id, 0xFE, 0x1E)
    
    # SET MOTORS ENABLE DELAY TIME = 100
    for a in range(0, 6):
        
        PWM_MSB, PWM_LSB = divmod(100, 1<<8)
        
        byte_low = PWM_ON_ADDR_LOW[a]
        byte_high = PWM_ON_ADDR_HIGH[a]
        
        wiringpi2.wiringPiI2CWriteReg8(pwm_id, byte_low, PWM_LSB)
        wiringpi2.wiringPiI2CWriteReg8(pwm_id, byte_high, PWM_MSB)
    
    return pwm_id



# --------------  PWM SCHEDULER FUNCTIONS  --------------

# CONVERSION [CONTROL MATRIX] -> [ESC PWM]

def set_time(control_list):

    for a in range(0, 6):
        timer_list[a] = int(919 + (819 * control_list[a]/100))

    return timer_list



# TURN ON MOTORS, SET TASKS FOR EVERY AXIS ENABLE TIME, TRIGGER IN 200Hz LOOP!

def set_times(pwm_id, times):

    for a in range(0, 6):
        
        PWM_MSB, PWM_LSB = divmod(times[a], 1<<8)
        
        byte_low = PWM_OFF_ADDR_LOW[a]
        byte_high = PWM_0FF_ADDR_HIGH[a]
        
        wiringpi2.wiringPiI2CWriteReg8(pwm_id, byte_low, PWM_LSB)
        wiringpi2.wiringPiI2CWriteReg8(pwm_id, byte_high, PWM_MSB)
