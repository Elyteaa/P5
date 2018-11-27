#!/usr/bin/python
import math
import numpy as np
from __future__ import print_function
import matplotlib.pyplot as plt
import sys

#For Points we use np.array

"""class WayPoints:

	def __init__(self):
		self.wayPoint = [] #array of all the waypoints

	#add a waypoint
	def addWayPoint(self, x, y):
		tempPoint = np.array([x, y])
		#tempPoint = np.transpose(tempPoint)
		self.wayPoint.append(tempPoint)

	#remove a waypoint """

def xor(*args):
	return sum(args) == 1

def AStarSearch(start, end, graph):
 
	G = {} #Actual movement cost to each position from the start position
	F = {} #Estimated movement cost of start to end going via this position
 
	#Initialize starting values
	G[start] = 0 
	F[start] = graph.heuristic(start, end)
 
	closedVertices = set()
	openVertices = set([start])
	cameFrom = {}
 
	while len(openVertices) > 0:
		#Get the vertex in the open list with the lowest F score
		current = None
		currentFscore = None
		for pos in openVertices:
			if current is None or F[pos] < currentFscore:
				currentFscore = F[pos]
				current = pos
 
		#Check if we have reached the goal
		if current == end:
			#Retrace our route backward
			path = [current]
			while current in cameFrom:
				current = cameFrom[current]
				path.append(current)
			path.reverse()
			return path, F[end] #Done!
 
		#Mark the current vertex as closed
		openVertices.remove(current)
		closedVertices.add(current)
 
		#Update scores for vertices near the current position
		for neighbour in graph.get_vertex_neighbours(current):
			if neighbour in closedVertices: 
				continue #We have already processed this node exhaustively
			candidateG = G[current] + graph.move_cost(current, neighbour)
 
			if neighbour not in openVertices:
				openVertices.add(neighbour) #Discovered a new vertex
			elif candidateG >= G[neighbour]:
				continue #This G score is worse than previously found
 
			#Adopt this G score
			cameFrom[neighbour] = current
			G[neighbour] = candidateG
			H = graph.heuristic(neighbour, end)
			F[neighbour] = G[neighbour] + H
 
	raise RuntimeError("A* failed to find a solution")
 
class AStarGraph(object):
	#Define a class board like grid with two barriers
 
	def __init__(self):
		self.barriers = []
		#The Steffen's tower base
		self.barriers.append([(2,2),(2,6),(5,6),(5,2),(2,2)])
 
	def heuristic(self, start, goal):
		#Use Chebyshev distance heuristic if we can move one square either
		#adjacent or diagonal
		D = 1
		D2 = 1
		dx = abs(start[0] - goal[0])
		dy = abs(start[1] - goal[1])
		return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)
 
	def get_vertex_neighbours(self, pos):
		n = []
		#Moves allow link a chess king
		for dx, dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1)]:
			x2 = pos[0] + dx
			y2 = pos[1] + dy
			if x2 < 0 or x2 > 7 or y2 < 0 or y2 > 7:
				continue
			n.append((x2, y2))
		return n

	def isBetween(self, a, b, c):
		crossproduct = (c[1] - a[1]) * (b[0] - a[0]) - (c[0] - a[0]) * (b[1] - a[1])

		"""#If it's equal to either of the dots, return True as well
		print('a', a, 'b', b, 'c', c)
		if a == c or b == c:
			print('recognized')
			return True"""

		# compare versus epsilon for floating point values, or != 0 if using integers
		if abs(crossproduct) > sys.float_info.epsilon:
			return False

		dotproduct = (c[0] - a[0]) * (b[0] - a[0]) + (c[1] - a[1])*(b[1] - a[1])
		if dotproduct < 0:
			return False

		squaredlengthba = (b[0] - a[0])*(b[0] - a[0]) + (b[1] - a[1])*(b[1] - a[1])
		if dotproduct > squaredlengthba:
			return False

		return True

	def isTheDot(self, barrier, x):
		for i in range(0,5):
			if barrier[i] == x:
				return True
		return False

	def move_cost(self, a, b):
		for barrier in self.barriers:
			if self.isTheDot(barrier, b) or xor(self.isBetween(barrier[0], barrier[1], b), self.isBetween(barrier[1], barrier[2], b), self.isBetween(barrier[2], barrier[3], b), self.isBetween(barrier[3], barrier[4], b)):
				return 100 #Extremely high cost to enter barrier squares
		return 1 #Normal movement cost

class PlanThePath:
#Add functions to find the next point, calculate needed values
   
	def __init__(self, W1, W2):
		self.W1 = W1
		self.W2 = W2
		self.u0 = np.array([0, 0])

	def robot_drive(self, x):
		#Move towards the x point

	def plan(self):
		atThePoint = False
		self.u0 = self.W2 - self.W1
		norm = np.linalg.norm(self.u0)
		self.u0 = np.divide(self.u0, norm)
		x = np.array([2, 2])
		u = np.array([1, 0])
		omf = 0.1
		dt = 0.1
		Robot = np.array([[0, -1], [1, 0]])

		tempCounter = 0
		while tempCounter <= 10:
			#Substitute with 'if not near the final position'
			tempCounter += 1
			#print(atThePoint)
			x = x + dt * u * omf
			self.robot_drive(x)
			xf = self.W2 + (omf - np.transpose(self.u0) * (self.W2 - x)) * self.u0
			v = xf - x
			v = v / np.linalg.norm(v)
			omd = (u[0] * v[1] - u[1] * v[0]) * 2
			R = Robot * dt
			ru = np.array([R[0][0] * u[0] + R[0][1] * u[1], R[1][0] * u[0] + R[1][1] * u[1]])
			ru = ru * omd
			u = u + ru