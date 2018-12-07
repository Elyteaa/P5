from __future__ import print_function
import matplotlib.pyplot as plt
import sys

def xor(*args):
	return sum(args) == 1

def isBetween(a, b, c):
	crossproduct = (c[1] - a[1]) * (b[0] - a[0]) - (c[0] - a[0]) * (b[1] - a[1])

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
			if x2 < -2000 or x2 > 2000 or y2 < -2000 or y2 > 2000:
				continue
			n.append((x2, y2))
		return n

	def isTheDot(self, barrier, x):
		for i in range(0,5):
			if barrier[i] == x:
				return True
		return False

	def move_cost(self, a, b):
		for barrier in self.barriers:
			if self.isTheDot(barrier, b) or xor(isBetween(barrier[0], barrier[1], b), isBetween(barrier[1], barrier[2], b), isBetween(barrier[2], barrier[3], b), isBetween(barrier[3], barrier[4], b)):
				return 100 #Extremely high cost to enter barrier squares
		return 1 #Normal movement cost
 
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
		print(current)
 
	raise RuntimeError("A* failed to find a solution")

def optimizeWaypoints(waypoints):
	if len(waypoints) > 2:
		for x in range(0,len(waypoints)-2):
			if isBetween(waypoints[x], waypoints[x+2], waypoints[x+1]):
				#print(waypoints[x], waypoints[x+2], waypoints[x+1])
				del waypoints[x+1]
				#print('new:', waypoints)
				return optimizeWaypoints(waypoints)
			elif x == len(waypoints)-3:
				return waypoints

if __name__=="__main__":
	graph = AStarGraph()
	result, cost = AStarSearch((-100,-100), (100,150), graph)
	print ("route", result)
	result = optimizeWaypoints(result)
	print ("route", result)
	print ("cost", cost)
	print ("first tuple", result[0][0])
	#plt.plot([v[0] for v in result], [v[1] for v in result])
	#for barrier in graph.barriers:
		#plt.plot([v[0] for v in barrier], [v[1] for v in barrier])
	#plt.xlim(-1,8)
	#plt.ylim(-1,8)
	#plt.show()