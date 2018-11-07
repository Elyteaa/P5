#!/usr/bin/python

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

class measurementToUseChooser:

	def __init__(self, measurementHistory, numSats, currentTime):
		self.measurementsUse = []
		self.usePosCalc = False
		self.measurementLoop = True
		self.count = 1
		self.compareID = numSats
		self.measurementsUse.append(measurementHistory[len(measurementHistory)-1])
		self.compareID = measurementHistory[len(measurementHistory)-1].transmitterID

		self.lookfor()

	def lookfor(self):
		while measurementLoop:
			count += 1

			tempID = measurementHistory[len(measurementHistory)-count].transmitterID
			measTime = measurementHistory[len(measurementHistory)-count].timestampMS
			measLvl = measurementHistory[len(measurementHistory)-count].ultrasoundLevel
			
			for n in range(0,numSats): #ask Jacob
				if tempID != compareID and compareID == 0 and currentTime < measTime + 5000 and measLvl >= 5:
					compareID = tempID
					measurementsUse.append(measurementHistory[len(measurementHistory)-count])
					break
				elif tempID == compareID or currentTime > measTime + 5000 or measLvl < 5:
					break

			if len(measurementsUse) >= numSats:
				usePosCalc = True
				measurementLoop = False
			elif count >= len(measurementHistory) or measurementsUse[0].timestampMS > measurementHistory[len(measurementHistory)-count].timestampMS + 1000:
				if len(measurementsUse) > 2
					usePosCalc = True
				else:
					usePosCalc = False
				measurementLoop = False