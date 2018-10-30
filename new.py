#!/usr/bin/python
import serial
import time
from array import *

def StateIdLe():
    global messageState, receiveCounter
    if int(z1serial.read(1).hex(),16) == startbyte:
        messageState = "StateData"
        receiveCounter = 0
    print('StateIdLe reached', receiveCounter)

def StateData():
    global messageState, newMeasReady, singleRead, receivedMessage, receiveCounter
    if int(z1serial.read(1).hex(),16) == DLEChar:
        messageState = "StateDataDLE"
    elif int(z1serial.read(1).hex(),16) == endbyte:
        messageState = "StateIdLe"
        newMeasReady = True
        singleRead = False
    else:
        receivedMessage[receiveCounter] = int(z1serial.read(1).hex(),16)
        receiveCounter = receiveCounter + 1
    print('StateData reached', receiveCounter)

def StateDataDLE():
    global receivedMessage, receiveCounter, messageState
    receivedMessage[receiveCounter] = int(z1serial.read(1).hex(),16) - 0x20 #types?
    receiveCounter = receiveCounter + 1
    messageState = "StateData"
    print('StateDataDLE reached', receiveCounter)

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
#print(z1serial.is_open)  # True for opened
if z1serial.is_open:
    while True:
        size = z1serial.inWaiting()
        if size:
            singleRead = True
            while singleRead:
                switchCase[messageState]()
                if newMeasReady:
                    newMeasReady = False
                    inputBuffer = [None]*receiveCounter
                    sum1 = 0
                    sum2 = 0
                    for i in range(1, len(inputBuffer)-2):
                        sum1 = int((sum1+(inputBuffer[i]) & 0xff)) % 255
                        #sum2 = (sum2 + sum1) % 255
                checksumcalculated = ((sum2 & 0xff) << 8) | (sum1 & 0xff)                
        else:
            print('no data')
        time.sleep(1)
else:
    print('z1serial not open')
#z1serial.close()  # close z1serial if z1serial is open.