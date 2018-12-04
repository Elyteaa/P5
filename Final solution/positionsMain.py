#!/usr/bin/python
import serial
import rospy
import time
from positionsClass import *
import math
from std_msgs.msg import Int32MultiArray

pub = rospy.Publisher('position', Int32MultiArray, queue_size=10)
rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(10) # 10hz

def StateIdLe():
    global messageState, receiveCounter
    if byteRead == startbyte:
        messageState = "StateData"
        print('hello')
        receiveCounter = 0

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
        receiveCounter += 1

def StateDataDLE():
    global receivedMessage, receiveCounter, messageState
    receivedMessage[receiveCounter] = byteRead - 0x20
    receiveCounter += 1
    messageState = "StateData"

switchCase = {
"StateIdLe": StateIdLe,
"StateData": StateData,
"StateDataDLE": StateDataDLE
}

#Callibration
numSats = 4
SatPosList = []
Satid2 = 42928
Satid4 = 42929
Satid1 = 42497
Satid3 = 42498
S1X = -500
S1Y = 0
S1Z = 0
S2X = 500
S2Y = 0
S2Z = 0
S3X = 0
S3Y = -500
S3Z = 0
S4X = 0
S4Y = 500
S4Z = 0

z1baudrate = 115200
z1port = '/dev/ttyUSB0'  # set the correct port before running it
measurementHistory = []
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
while not rospy.is_shutdown():
    if z1serial.is_open:
        while True:
            size = z1serial.inWaiting()
            if size > 255:
                z1serial.reset_input_buffer()
            else:
                if len(measurementHistory) > 10:
                    #measurementHistory.clear()
                    del measurementHistory[:]
                singleRead = True
                while singleRead:
                    byteRead = z1serial.read(1)
                    #byteRead = bytes(byteRead, 'utf-8')
                    #byteRead = bytearray(byteRead).hex()
                    #byteRead = codecs.decode(z1serial.read(1).strip, 'hex')
                    byteRead = (ord(byteRead))#bytes(z1serial.read(1))#int(z1serial.read(1).hex(),16)
                    #byteRead = byteRead % 255
                    #byteRead = int(byteRead.hex(), 16)
                    #byteRead = bytes.fromhex(byteRead2)
                    #print('print = ', byteRead, 'type = ', type(byteRead))
                    #print("print = ", byteRead, "type = ", type(byteRead))
                    #byteRead2 = bytes(z1serial.read(1))
                    #byy = int(byteRead2, 16)
                    #print(type(byy))
                    switchCase[messageState]()
                if newMeasReady:
                    newMeasReady = False
                    for i in range(0, receiveCounter):
                        inputBuffer[i] = receivedMessage[i]
                    sum1 = 0
                    sum2 = 0
                    checksumcalculated = 0
                    for i in range(1, receiveCounter-2):
                        sum1 = (sum1+(inputBuffer[i] & 0xff)) % 255
                        sum2 = (sum2 + sum1) % 255
                    checksumcalculated = ((sum2 & 0xff) << 8) | (sum1 & 0xff)

                    if inputBuffer[0] == 12 and inputBuffer[1] == 1:
                        incomingMeasurement = CPRMeasurement(0, 0, 0, 0, 0, 0, 0, 0)
                        incomingMeasurement.timestampMS = int(round(time.time()*1000))
                        incomingMeasurement.numMaster = (inputBuffer[6] & 0xff)
                        incomingMeasurement.CPRID = ((inputBuffer[4] & 0xff) << 16) | ((inputBuffer[3] & 0xff) << 8) | ((inputBuffer[2] & 0xff))

                        checksum = (inputBuffer[14] & 0xff) << 8 | (inputBuffer[13] & 0xff)
                        if checksum == checksumcalculated and incomingMeasurement.CPRID > 0:
                            incomingMeasurement.ultrasoundLevel = inputBuffer[5] & 0xff
                            incomingMeasurement.RSSI = inputBuffer[7] & 0xff
                            incomingMeasurement.timeDifference = ((inputBuffer[12] & 0xff) << 8) | (inputBuffer[11] & 0xff)
                            incomingMeasurement.transmitterID = ((inputBuffer[10]) & 0xff) << 16 | (inputBuffer[9] & 0xff) << 8 | (inputBuffer[8] & 0xff)

                            measurement = trueMeasurement(incomingMeasurement.transmitterID, 0, incomingMeasurement.RSSI, incomingMeasurement.ultrasoundLevel, incomingMeasurement.timestampMS, incomingMeasurement.CPRID, incomingMeasurement.timeDifference, 21)
                            measurementHistory.append(measurement)

                            ID1 = trueMeasurement(0, 0, 0, 0, 0, 0, 0, 0)
                            ID2 = trueMeasurement(0, 0, 0, 0, 0, 0, 0, 0)
                            ID3 = trueMeasurement(0, 0, 0, 0, 0, 0, 0, 0)
                            ID4 = trueMeasurement(0, 0, 0, 0, 0, 0, 0, 0)
                            d12 = 0
                            d34 = 0
                            stop = False
                            n = 1
                            
                            if len(measurementHistory):
                                while not stop and n < len(measurementHistory):
                                    n += 1
                                    if ID1.transmitterID == 0 and measurementHistory[len(measurementHistory)-n].transmitterID == Satid1:
                                        ID1 = measurementHistory[len(measurementHistory)-n]
                                        d12 = d12 + abs(S1X)
                                        print(ID1.distance)
                                    if ID2.transmitterID == 0 and measurementHistory[len(measurementHistory)-n].transmitterID == Satid2:
                                        ID2 = measurementHistory[len(measurementHistory)-n]
                                        d12 = d12 + abs(S2X)
                                    if ID3.transmitterID == 0 and measurementHistory[len(measurementHistory)-n].transmitterID == Satid3:
                                        ID3 = measurementHistory[len(measurementHistory)-n]
                                        d34 = d34 + abs(S3Y)
                                    if ID4.transmitterID == 0 and measurementHistory[len(measurementHistory)-n].transmitterID == Satid4:
                                        ID4 = measurementHistory[len(measurementHistory)-n]
                                        d34 = d34 + abs(S4Y)
                                    if ID4.transmitterID and ID3.transmitterID and ID2.transmitterID and ID1.transmitterID:
                                        a12 = (ID1.distance**2 - ID2.distance**2) / (2 * d12**2) + 0.5
                                        a34 = (ID3.distance**2 - ID4.distance**2) / (2 * d34**2) + 0.5
                                        xx = d12 * (2 * a12 - 1)
                                        xy = d34 * (2 * a34 - 1)
                                        #print('xx = ', xx, 'xy = ', xy)
                                        xx = xx/2
                                        xy = xy/2
                                        coordset =Int32MultiArray()
                                        coordset.data = [xx/10, xy/10]
                                        stop = True
                                        rospy.loginfo(coordset)
                                        pub.publish(coordset)
                                        rate.sleep()
                                        #rospy.spin()
                            newMeasGood = True

                        else:
                            newMeasGood = False
    else:
        print('z1serial not open')
#if __name__ == '__main__':
#    try:
#        talker()
#    except rospy.ROSInterruptException:
#        pass
