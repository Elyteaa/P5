#!/usr/bin/python

import math
import numpy as np

#For Points we use np.array

class WayPoints:

	def __init__(self):
		self.wayPoint = [] #array of all the waypoints

	#add a waypoint
	def addWayPoint(self, x, y):
		tempPoint = np.array([x, y])
		#tempPoint = np.transpose(tempPoint)
		self.wayPoint.append(tempPoint)

	#remove a waypoint

class PlanThePath:
#Add functions to find the next point, calculate needed values
   
	def __init__(self, W1, W2):
		self.W1 = W1
		self.W2 = W2
		self.u0 = np.array([0, 0])

	def plan(self):
		atThePoint = False
		self.u0 = self.W2 - self.W1
		norm = np.linalg.norm(self.u0)
		self.u0 = np.divide(self.u0, norm)
		x = np.array([2, 2])
		#x = np.transpose(x)
		u = np.array([1, 0])
		#u = np.transpose(u)
		omf = 0.1
		dt = 0.1
		Robot = np.array([[0, -1], [1, 0]])

		tempCounter = 0
		while tempCounter <= 10:
			#If not near the final position
			tempCounter += 1
			#print(atThePoint)
			#print('x + dt', x+dt)
			#print('dt * u', dt * u)
			#print('dt * u * omf', dt * u * omf)
			x = x + dt * u * omf
			#print('x + revious', x)
			xf = self.W2 + (omf - np.transpose(self.u0) * (self.W2 - x)) * self.u0
			#print(xf)
			v = xf - x
			v = v / np.linalg.norm(v)
			omd = (u[0] * v[1] - u[1] * v[0]) * 2
			R = Robot * dt
			ru = np.array([R[0][0] * u[0] + R[0][1] * u[1], R[1][0] * u[0] + R[1][1] * u[1]])
			ru = ru * omd
			#print('ru', ru, 'u', u)
			print(u + ru)
			#print(ru)
			#u = u + dt * ru * u * omd
			#print(R * u)
			#print(R[0][0], R[0][1], R[1][0], R[1][1])
			#print('u =', u)