#!/usr/bin/python
from __future__ import print_function
import math
import numpy as np
import matplotlib.pyplot as plt
import sys

def xor(*args):
    return sum(args) == 1

def inCircle(center, radius, point):
    if (point[0] - center[0])**2 + (point[1] - center[1])**2 < radius**2:
        return True
    return False

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
        self.barriers.append([(402, -627),(-554,-582),(-435,700),(638,419),(402, -627)])
 
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
            if x2 < -200 or x2 > 200 or y2 < -200 or y2 > 200:
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

class PlanThePath:
#Add functions to find the next point, calculate needed values
   
    def __init__(self, path):
        self.waypoints = path       

    def robot_drive(self, x):
        #Move towards the x point
        pass

    def move(self):
        atThePoint = False
        for n in range(0,len(self.waypoints) - 1):
            self.u0 = self.waypoints[n] - self.waypoints[n+1]
            norm = np.linalg.norm(self.u0)
            self.u0 = np.divide(self.u0, norm)
            x = np.array([600,600])
            u = np.array([1, 0])
            #wanted speed of the robot
            omf = 0.1
            #still don't know what this is
            dt = 0.1
            #or this
            Robot = np.array([[0, -1], [1, 0]])
            while not atThePoint:
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
                #If the robot's position is within 10 centimeters from the goal, we move on
                if inCircle(self.waypoints[n+1], 100, x):
                    atThePoint = True
