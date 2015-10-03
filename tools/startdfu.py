import sys
import serial

s = serial.Serial(sys.argv[1], 3000000, timeout=0)
s.write("__dfu\r")
while True:
    b = s.read(1)
    if b == "\a":
        print "Device rejected DFU entry command"
        sys.exit()
