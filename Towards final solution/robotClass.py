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
		u = np.array([1, 0])
		omf = 0.1
		dt = 0.1
		R = np.array([[0, -1], [1, 0]])

		while not atThePoint:
			#If not near the final position
			print(atThePoint)
			atThePoint = True