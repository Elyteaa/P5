#!/usr/bin/python
import serial
import time
from array import *
#from ctypes import *

#messageState = "StateIdLe"

def StateIdLe():
    if int(z1serial.read(1).hex(),16) == startbyte:
        #messageState = "StateData"
        receiveCounter = 0
        print('start byte')
    else:
        print('not start byte')
    #return (receiveCounter)

switchCase = {
"StateDataDLE": StateIdLe,
}

z1baudrate = 115200
z1port = '/dev/ttyUSB0'  # set the correct port before running it
data = []
receiveCounter = 0
receivedMessage = [None]*255
startbyte = 0x02
messageState = "StateDataDLE"

z1serial = serial.Serial(port=z1port, baudrate=z1baudrate, bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE)
z1serial.timeout = None  # set read timeout
# print z1serial  # debug serial.
print(z1serial.is_open)  # True for opened
if z1serial.is_open:
    while True:
        size = z1serial.inWaiting()
        if size:
            singleRead = True
            while True:
                switchCase[messageState]()
                #data = int(z1serial.read(1).hex(), 16)
                
                """if data == startbyte:
                    print(data)
                else:
                    print('no start byte yet')"""
            #print(data[0])
        else:
            print('no data')
        time.sleep(1)
else:
    print('z1serial not open')
# z1serial.close()  # close z1serial if z1serial is open.

while True:
	if data == x00:
		print("start")
		
	else:
		print(":(")
