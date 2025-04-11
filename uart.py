import serial
import Adafruit_BBIO.UART as UART
import time
import random

'''
ser = serial.Serial('/dev/tty01', 9600)
while True:
    line = ser.readline()
    if line:
        print(line)
'''

#v = str(random.randint(1,10)/20)
#w = str(random.randint(1,10)/20)

UART.setup("UART5")

serial = serial.Serial(port = "/dev/ttyO5", baudrate=9600)
serial.close()
serial.open()
count = 0
while True:
    serial.write(b'!0.3@0.014#')
    time.sleep(0.1)
    count = count + 1
    if count == 50:
        break
serial.write(b'!0@0#')
serial.close()

