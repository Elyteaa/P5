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
		self.measurementsUse.append(measurementHistory[len(measurementHistory)-1])
		self.lookfor(measurementHistory, numSats, currentTime)

	def lookfor(self, measurementHistory, numSats, currentTime):
		count = 1
		compareID = measurementHistory[len(measurementHistory)-1].transmitterID
		measurementLoop = True
		while measurementLoop:
			count += 1

			tempID = measurementHistory[len(measurementHistory)-count].transmitterID
			measTime = measurementHistory[len(measurementHistory)-count].timestampMS
			measLvl = measurementHistory[len(measurementHistory)-count].ultrasoundLevel
			
			for n in range(0,numSats): #ask Jacob
				if tempID != compareID and compareID == 0 and currentTime < measTime + 5000 and measLvl >= 5:
					compareID = tempID
					self.measurementsUse.append(measurementHistory[len(measurementHistory)-count])
					break
				elif tempID == compareID or currentTime > measTime + 5000 or measLvl < 5:
					break

			if len(self.measurementsUse) >= numSats:
				self.usePosCalc = True
				measurementLoop = False
			elif count >= len(measurementHistory) or self.measurementsUse[0].timestampMS > measurementHistory[len(measurementHistory)-count].timestampMS + 1000:
				if len(self.measurementsUse) > 2:
					self.usePosCalc = True
				else:
					self.usePosCalc = False
				measurementLoop = False

class ForwardCalculation:
	def __init__(self, measHis, measurementToUse, currentTime):
		self.measurementForward = measurementToUse
		self.prediction(measHist, currentTime, 2)
	def prediction(self, measHist, currentTime, numPrev):
		if numPrev > len(measHist):
            numPrev = len(measHist)

        measSize = len(self.measurementForward)
        speed = [0] * measSize
        findPrevDist = True
        prevDist = [[[ 0 for col in range(measSize)] for row in range(numPrev+1)] for layers in range(2)]
        
        for n in range(measSize):
            count2 = 0
            measID = self.measurementForward[n].transmitterID
            count = len(measHist)-1
            while findPrevDist:
                if measID == measHist[count].transmitterID && count2 < numPrev + 1 && measHist[count].ultrasoundLevel > 5:
                    prevDist[n][count2][0] = measHist[count].distance
                    prevDist[n][count2][1] = measHist[count].timestampMS
                    count2 += 1
                    prevDistCount = count2
                if count == 0 || count2 >= numPrev + 1:
                	break
                count -= 1
            speed[n] = 0
            if count2 == 2:
            	speed[n] = (((prevDist[n][0][0]) - (prevDist[n][count2 - 1][0])) / (prevDist[n][0][1] - prevDist[n][count2 - 1][1]))
            elif count2 == 3:
            	speed2 = (prevDist[n][0][0] - prevDist[n][1][0]) / (prevDist[n][0][1] - prevDist[n][1][1])
            	speed1 = (prevDist[n][1][0] - prevDist[n][2][0]) / (prevDist[n][1][1] - prevDist[n][2][1])
            	speed[n] = ((speed2-speed1)/(prevDist[n][0][1]-prevDist[n][2][1]))*(currentTime- prevDist[n][0][1])

			if speed[n] > 2:
				speed[n] = 0
			elif speed[n] < 0.05:
				speed[n] = 0

		for n in range(measSize):
			speedCounter = 0
			for i in range(measSize):
				if speed[n] != 0 && speed[i] == 0:
					speedCounter += 1
			if speedCounter > 2:
				speed[n] = 0
			if count2 > 1:
				timeDiff = currentTime - prevDist[n][0][1]
				tempMeas = self.measurementForward[n]
				tempMeas.distance = tempMeas.distance + speed[n]*timeDiff
				self.measurementForward[n] = tempMeas