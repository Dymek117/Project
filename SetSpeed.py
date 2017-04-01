# -*- coding: utf-8 -*-
import wiringpi2
import time

pwm_address = 0b00111110   # A6,A5 TO GND, A4-A0 TO VCC



# SET SPEED REGISTERS ADDRESS

north_east = 0x06    # 1ST, TOP ROW
north_west = 0x07    # 2nd, TOP ROW
west =       0x05    # 3rd, TOP ROW
south_west = 0x04    # 1st, BOTTOM ROW
south_east = 0x03    # 2nd, BOTTOM ROW
east =       0x02    # 3rd, BOTTOM ROW


# SPEED VALUES [ % OF CYCLE TIME 1-256 ]

north_east = 0
north_west = 0
west =       0
south_west = 0
south_east = 0
east =       0

# BLINKING PERIOD LENGTH ADJUST
`
