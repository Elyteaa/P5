#!/usr/bin/python
import math

class point3D:

	def __init__(self, x, y, z):
		self.x = x
		self.y = y
		self.z = z

class SatellitePositions:

	def __init__(self, masterpos, ID, position):
		self.masterpos = masterpos
		self.transmitterID = ID
		self.position = position


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
		#print('lookfor entered ', len(measurementHistory))
		count = 1
		compareID = [0] * numSats
		compareID[0] = measurementHistory[len(measurementHistory)-1].transmitterID
		measurementLoop = True
		self.measurementsUse.append(measurementHistory[len(measurementHistory)-1])
		while measurementLoop:
			count += 1

			tempID = measurementHistory[len(measurementHistory)-count].transmitterID
			measTime = measurementHistory[len(measurementHistory)-count].timestampMS
			measLvl = measurementHistory[len(measurementHistory)-count].ultrasoundLevel
			
			for n in range(0,numSats):
				#print('n in numSats: ', numSats)
				if tempID != compareID[n] and compareID[n] == 0 and currentTime < measTime + 5000 and measLvl >= 5:
					print('compareID values: ', compareID)
					compareID[n] = tempID
					self.measurementsUse.append(measurementHistory[len(measurementHistory)-count])
					break
				elif tempID == compareID[n] or currentTime > measTime + 5000 or measLvl < 5:
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

	def __init__(self, measHist, measurementToUse, currentTime):
		self.measurementForward = measurementToUse
		self.prediction(measHist, currentTime, 2)

	def prediction(self, measHist, currentTime, numPrev):
		if numPrev > len(measHist):
			numPrev = len(measHist)

		measSize = len(self.measurementForward.measurementsUse)
		speed = [0] * measSize
		findPrevDist = True
		prevDist = [[[ 0 for col in range(measSize)] for row in range(numPrev+1)] for layers in range(2)]

		count2 = [0] * measSize
		for n in range(measSize):
			count2[n] = 0
			measID = self.measurementForward.measurementsUse[n].transmitterID
			count = len(measHist)-1
			while findPrevDist:
				#count2 = 0
				#print('for n', measSize, 'for count2[n]', numPrev+1, 'for count', len(measHist))
				#print('n=   ', n,        'count2[n]=   ', count2[n], 'count=   ', count)
				if measID == measHist[count].transmitterID and count2[n] < numPrev + 1 and measHist[count].ultrasoundLevel > 5:
					#print('n=   ', n,        'count2[n]=   ', count2[n], 'count=   ', count)
					prevDist[0][count2[n]][n] = measHist[count].distance
					prevDist[1][count2[n]][n] = measHist[count].timestampMS
					count2[n] += 1
					prevDistCount = count2[n]
				if count == 0 or count2[n] >= numPrev + 1:
					break
				count -= 1
			speed[n] = 0
			if count2[n] == 2:
				speed[n] = (((prevDist[0][0][n]) - (prevDist[0][count2[n] - 1][n])) / (prevDist[1][0][n] - prevDist[1][count2[n] - 1][n]))
			elif count2[n] == 3:
				speed2 = (prevDist[0][0][n] - prevDist[0][1][n]) / (prevDist[1][0][n] - prevDist[1][1][n])
				speed1 = (prevDist[0][1][n] - prevDist[0][2][n]) / (prevDist[1][1][n] - prevDist[1][2][n])
				speed[n] = ((speed2-speed1)/(prevDist[1][0][n]-prevDist[1][2][n]))*(currentTime- prevDist[1][0][n])
			if speed[n] > 2:
				speed[n] = 0
			elif speed[n] < 0.05:
				speed[n] = 0
		for n in range(measSize):
			speedCounter = 0
			for i in range(measSize):
				if speed[n] != 0 and speed[i] == 0:
					speedCounter += 1
			if speedCounter > 2:
				speed[n] = 0
			if count2[n] > 1:
				timeDiff = currentTime - prevDist[1][0][n]
				tempMeas = self.measurementForward.measurementsUse[n]
				tempMeas.distance = tempMeas.distance + speed[n]*timeDiff
				self.measurementForward.measurementsUse[n] = tempMeas

class FindSatellitePos:

	def __init__(self, measurementForward, SatPosList):
		self.SatPosUse = []
		self.calculate(measurementForward, SatPosList)

	def calculate(self, measurementForward, SatPosList):
		for nn in range(len(measurementForward.measurementForward.measurementsUse)):
			for n in range(len(SatPosList)):
				if measurementForward.measurementForward.measurementsUse[nn].transmitterID == SatPosList[n].transmitterID:
					self.SatPosUse.append(SatPosList[n])
					break

class TrilaterateManyLinearEquations:

	def __init__(self, SatPosUse, measurementsUse, measForwSize):
		self.result = 0
		self.themath(SatPosUse, measurementsUse, measForwSize)

	def themath(self, SatPosUse, measurementsUse, measForwSize):
		counter = 0
		for n in range(measForwSize):
			for nn in range(n+1, measForwSize):
				for nnn in range(nn+1,measForwSize):
					if n != nn and nn != nnn and n != nnn:
						aPosition = SatPosUse[n]
						bPosition = SatPosUse[nn]
						cPosition = SatPosUse[nnn]
						aDistance = measurementsUse[n]
						bDistance = measurementsUse[nn]
						cDistance = measurementsUse[nnn]

						temp = math.sqrt((bPosition.x -aPosition.x)**2 + (bPosition.y-aPosition.y)**2 + (bPosition.z-aPosition.z)**2)
						print(temp)
						#ex = [[aPosition.x * 1/temp, aPosition.y * 1/temp, aPosition.z * 1/temp], [bPosition.x * 1/temp, bPosition.y * 1/temp, bPosition.z * 1/temp]]
						i = aPosition.x * cPosition.x + aPosition.y * cPosition.y + aPosition.z * cPosition.z
						"""for x in range(2):
							for y in range(3):
								temp2[x][y] = ex[x][y] * i"""
						#iteration of lists may be Jewish
						"""for n in range(3):
							for x in range(2):
								for y in range(3):
									temp = temp + ((cPosition[n] - ex[x][y] * i) - (aPosition[n] - ex[x][y]*i))**2"""
						#temp = [[aPosition[0] - temp2, aPosition[1] - temp, aPosition[2] - temp], [cPosition[0] - temp, cPosition[1] - temp, cPosition[2] - temp]]
						#temp = math.sqrt((temp2[0][1] - temp2[0][0])**2 + (temp2[1][1] - temp2[1][0])**2 + (temp2[2][1] - temp2[2][0])**2)
						for x in range(2):
							for y in range(3):
								exi[x][y] = ex[x][y] * i
						temp = [[aPosition.x - exi[0][0], aPosition.y - exi[0][1], aPosition.z - exi[0][2]], [cPosition.x - exi[1][0], cPosition.y - exi[1][1], cPosition.z - exi[1][2]]]
						temp = math.sqrt((cPosition.x-aPosition.x)**2 + (cPosition.y-aPosition.y)**2 + (cPosition[2]-aPosition[2])**2)
						ey = [[aPosition.x-exi[0][0], aPosition.y-exi[0][1], aPosition.z-exi[0][2]], [cPosition.x-exi[1][0], cPosition.y-exi[1][1], cPosition.z-exi[1][2]]]
						for x in range(2):
							for y in range(3):
								ey[x][y] = ey[x][y] * 1 / temp
						ez = [[ex[0][1] * ey[0][2] - ex[0][2] * ey[0][1], ex[0][2] * ey[0][0] - ex[0][0] * ey[0][2], ex[0][0] * ey[0][1] - ex[0][1] * ey[0][0]], [ex[1][1] * ey[1][2] - ex[1][2] * ey[1][1], ex[1][2] * ey[1][0] - ex[1][0] * ey[1][2], ex[1][0] * ey[1][1] - ex[1][1] * ey[1][0]]]
						d = math.sqrt((bPosition.x - aPosition.x)**2 + (bPosition.y - aPosition.y)**2 + (bPosition.z - aPosition.z)**2)
						j = aPosition.x * cPosition.x + aPosition.y * cPosition.y + aPosition.z * cPosition.z

						x = (aDistance**2 - bDistance**2 + d**2) / (2 * d)
						y = ((aDistance**2 - cDistance**2 + i**2 + j**2) / (2 * j)) - ((i / j) * x)
						z = math.sqrt(aDistance**2 - x**2 - y**2)

						if math.isnan(z) == False and math.isinf(z) == False:
							pass
							#temp = aPosition[2] + ex[]
							#if abs(aPosition[])