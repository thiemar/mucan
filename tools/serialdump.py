import sys
import serial

s = serial.Serial(sys.argv[1], 3000000, timeout=0)
s.write("O\r")
while True:
    b = s.read(1)
    if b:
        if b == "\r":
            sys.stdout.write("\n")
        else:
            sys.stdout.write(b)
