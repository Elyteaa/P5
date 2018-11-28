#!/usr/bin/python
import math
import numpy as np
import time #may be not needed
from robotClass import *

#map1 = WayPoints() #the object that stores all way points of a map

#map1.addWayPoint(1, 0)
#map1.addWayPoint(0, 1)

#print(map1.wayPoint[0])

#path = PlanThePath(map1.wayPoint[0], map1.wayPoint[1])
#print(path.W1, path.W2)

def optimizeWaypoints(waypoints):
	if len(waypoints) > 2:
		for x in range(0,len(waypoints)-2):
			if isBetween(waypoints[x], waypoints[x+2], waypoints[x+1]):
				#print(waypoints[x], waypoints[x+2], waypoints[x+1])
				del waypoints[x+1]
				#print(waypoints)
				return optimizeWaypoints(waypoints)

graph = AStarGraph()
start = (600, 600)
end = (700, 700)
result, cost = AStarSearch(start, end, graph)
result = optimizeWaypoints(result)
path = PlanThePath(result)
path.move()