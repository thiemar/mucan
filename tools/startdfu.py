from __future__ import print_function, unicode_literals

import sys
import serial

s = serial.Serial(sys.argv[1], 3000000, timeout=0)
s.write(b"__dfu\r")

try:
    while True:
        b = s.read(1)
        if b == b"\a":
            print("Device rejected DFU entry command")
            sys.exit()
except serial.serialutil.SerialException:
    print("Device entering DFU")
