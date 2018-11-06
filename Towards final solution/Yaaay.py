#!/usr/bin/python
import serial
import time
from array import *
from classModule import CPRMeasurement


def StateIdLe():
    global messageState, receiveCounter
    if byteRead == startbyte:
        messageState = "StateData"
        receiveCounter = 0
    #print('StateIdLe reached', receiveCounter)

def StateData():
    global messageState, newMeasReady, singleRead, receivedMessage, receiveCounter
    
    if byteRead == DLEChar:
        messageState = "StateDataDLE"
    elif byteRead == endbyte:
        messageState = "StateIdLe"
        newMeasReady = True
        singleRead = False
    else:
        receivedMessage[receiveCounter] = byteRead
        receiveCounter = receiveCounter + 1
    #print('StateData reached', receiveCounter)

def StateDataDLE():
    global receivedMessage, receiveCounter, messageState
    receivedMessage[receiveCounter] = byteRead - 0x20 #types?
    receiveCounter = receiveCounter + 1
    messageState = "StateData"
    #print('StateDataDLE reached', receiveCounter)

switchCase = {
"StateIdLe": StateIdLe,
"StateData": StateData,
"StateDataDLE": StateDataDLE
}

z1baudrate = 115200
z1port = '/dev/ttyUSB0'  # set the correct port before running it
data = []
receiveCounter = 0
receivedMessage = [0]*255
inputBuffer = [0]*255
startbyte = 0x02
DLEChar = 0x10
endbyte = 0x03
messageState = "StateIdLe"
newMeasReady = False

z1serial = serial.Serial(port=z1port, baudrate=z1baudrate, bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE)
z1serial.timeout = None  # set read timeout
# print z1serial  # debug serial.
#print(z1serial.is_open)  # True for opened
if z1serial.is_open:
    while True:
        size = z1serial.inWaiting()
        if size > 255:
            z1serial.reset_input_buffer()
            #print('no data')
        else:
            singleRead = True
            while singleRead:
                byteRead = int(z1serial.read(1).hex(),16)
                switchCase[messageState]()
                if newMeasReady:
                    newMeasReady = False
                    #inputBuffer = [0]*receiveCounter
                    for i in range(0, receiveCounter):
                        inputBuffer[i] = receivedMessage[i]
                        #print(i, ' ', inputBuffer[i])
                    sum1 = 0
                    sum2 = 0
                    checksumcalculated = 0
                    for i in range(1, receiveCounter-2):
                        sum1 = (sum1+(inputBuffer[i] & 0xff)) % 255
                        sum2 = (sum2 + sum1) % 255
                    checksumcalculated = ((sum2 & 0xff) << 8) | (sum1 & 0xff)

                    if inputBuffer[0] == 12 and inputBuffer[1] == 1:
                        print(inputBuffer[0])
                        incomingMeasurement = CPRMeasurement(0, 0, 0, 0, 0, 0, 0, 0)
                        incomingMeasurement.timestampMS = int(round(time.time()*1000))
                        incomingMeasurement.numMaster = (inputBuffer[6] & 0xff)
                        incomingMeasurement.CPRID = ((inputBuffer[4] & 0xff) << 16) | ((inputBuffer[3] & 0xff) << 8) | ((inputBuffer[2] & 0xff))

                        checksum = (inputBuffer[14] & 0xff) << 8 | (inputBuffer[13] & 0xff)
                        if checksum == checksumcalculated and incomingMeasurement.CPRID > 0:
                            incomingMeasurement.ultrasoundLevel = inputBuffer[5] & 0xff
                            #The following would be performed in for loop if more masters are present
                            incomingMeasurement.RSSI = (inputBuffer[7] & 0xff)
                            incomingMeasurement.timeDifference = ((inputBuffer[12] & 0xff) << 8) | (inputBuffer[11] & 0xff)
                            incomingMeasurement.transmitterID = ((inputBuffer[10]) & 0xff) << 16 | (inputBuffer[9] & 0xff) << 8 | (inputBuffer[8] & 0xff)

                        print(incomingMeasurement.ultrasoundLevel)




        time.sleep(1)
else:
    print('z1serial not open')
#z1serial.close()  # close z1serial if z1serial is open.
