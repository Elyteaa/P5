#!/usr/bin/python

import math
import numpy as np

class WayPoints:

	def __init__(self):
		self.wayPoint = [] #array of all the waypoints

	#add a waypoint
	def addWayPoint(self, x, y):
		tempPoint = (x, y)
		self.wayPoint.append(tempPoint)

	#remove a waypoint