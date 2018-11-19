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
		self.u0 = self.W2 - self.W1
		norm = np.linalg.norm(self.u0)
		self.u0 = np.divide(self.u0, norm)

		