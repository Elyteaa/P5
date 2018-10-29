#!/usr/bin/python
import serial
import time
from array import *

def StateIdLe():
    global messageState, receiveCounter
    if int(z1serial.read(1).hex(),16) == startbyte:
        messageState = "StateData"
        receiveCounter = 0

def StateData():
    global messageState, newMeasReady, singleRead, receivedMessage, receiveCounter
    if int(z1serial.read(1).hex(),16) == DLEChar:
        messageState = "StateDataDLE"
    else if int(z1serial.read(1).hex(),16) == endbyte:
        messageState = "StateIdLe"
        newMeasReady = True
        singleRead = False
    else:
        receivedMessage[receiveCounter] = int(z1serial.read(1).hex(),16)
        receiveCounter = receiveCounter + 1

def StateDataDLE():
    global receivedMessage, receiveCounter, messageState
    receivedMessage[receiveCounter] = int(z1serial.read(1).hex(),16) - 0x20 #types?
    receiveCounter = receiveCounter + 1
    messageState = "StateData"

switchCase = {
"StateIdLe": StateIdLe,
"StateData": StateData,
"StateDataDLE": StateDataDLE
}

z1baudrate = 115200
z1port = '/dev/ttyUSB0'  # set the correct port before running it
data = []
receiveCounter = 0
receivedMessage = [None]*255
startbyte = 0x02
DLEChar = 0x10
endbyte = 0x03
messageState = "StateDataDLE"
newMeasReady = False

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
                if newMeasReady:
                    newMeasReady = False
                    inputBuffer = [None]*255
                
        else:
            print('no data')
        time.sleep(1)
else:
    print('z1serial not open')
#z1serial.close()  # close z1serial if z1serial is open.