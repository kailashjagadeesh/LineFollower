import serial
import threading
import sys
port = str(sys.argv[1])
print ("connecting to: " + port)
bt = serial.Serial(port, 9600)

while (1):
    try:
        line = bt.readline().decode()
        print (line)

    except Exception:
        print ("IDK WHAT HAPPENED")

bt.close()