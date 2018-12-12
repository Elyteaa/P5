#!/usr/bin/python
#~
class CPRMeasurement:

    def __init__(self, CPRID, ultrasoundLevel, numMasters, timeDifference, timestampMS, transmitterID, masterID, RSSI):
        self.CPRID = CPRID
        self.ultrasoundLevel = ultrasoundLevel
        self.numMasters = numMasters
        self.timeDifference = timeDifference
        self.timestampMS = timestampMS
        self.transmitterID = transmitterID
        self.masterID = masterID
        self.RSSI = RSSI


class trueMeasurement: #temperature is needed here
    
    #data of incomngMeasurement
    def __init__(self, transmitterID, masterID, RSSI, ultrasoundLevel, timestampMS, CPRID, timeDifference, temperature):
        self.timestampMS = timestampMS
        self.receiverID = CPRID
        self.ultrasoundLevel = ultrasoundLevel
        self.distance = (timeDifference * (331.4+0.6*temperature)/1000) + 18
        self.transmitterID = transmitterID
        self.RSSI = RSSI
        self.masterID = masterID