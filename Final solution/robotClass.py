#!/usr/bin/python
from __future__ import print_function
from __future__ import division
from di_sensors.inertial_measurement_unit import InertialMeasurementUnit
import math
import time
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
 
class IMU:

    def __init__(self):
        self.imu = InertialMeasurementUnit(bus = "GPG3_AD1") #RPI_1 GPG3_AD1

    def getHeading(self):
        #Read the magnetometer, gyroscope, accelerometer, euler, and temperature values
        mag   = self.imu.read_magnetometer()
        gyro  = self.imu.read_gyroscope()
        accel = self.imu.read_accelerometer()
        euler = self.imu.read_euler()
        temp  = self.imu.read_temperature()

        """string_to_print = "Magnetometer X: {:.1f}  Y: {:.1f}  Z: {:.1f} " \
                          "Gyroscope X: {:.1f}  Y: {:.1f}  Z: {:.1f} " \
                          "Accelerometer X: {:.1f}  Y: {:.1f} Z: {:.1f} " \
                          "Euler Heading: {:.1f}  Roll: {:.1f}  Pitch: {:.1f} " \
                          "Temperature: {:.1f}C".format(mag[0], mag[1], mag[2],
                                                        gyro[0], gyro[1], gyro[2],
                                                        accel[0], accel[1], accel[2],
                                                        euler[0], euler[1], euler[2],
                                                        temp)"""
        
        #angle = math.atan2(accel[1], accel[0])
        #xyu = math.degrees(angle)
        #print(string_to_print)
        #print("angle first = ", xyu)
        #anglez = (180/math.pi * math.atan2(accel[0], accel[1]) % 360)
        #print("angle z = ", anglez)
        angley = (180/math.pi * math.atan2(mag[0], mag[2]) % 360)
        print("angle y = ", angley)
        #anglex = (180/math.pi * math.atan2(accel[1], accel[2]) % 360)
        #print("angle x = ", anglex)
        #angle1 = (180/math.pi * math.atan2(accel[1], accel[0]) % 360)
        #angle1 = (180/math.pi * math.atan2(accel[1], accel[0]) % 360)
        #print("angle 2 = ", angle1)
        return angley



class AStarGraph(object):
    #Define a class board like grid with two barriers
 
    def __init__(self):
        self.barriers = []
        #The Steffen's tower base
        self.barriers.append([(25, -25),(-25,-25),(-25,25),(25,25),(25, -25)])
 
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

    def robot_drive(self, heading):
        #Move towards the x point
        pass

    def move(self):
        atThePoint = False
        for n in range(0,len(self.waypoints) - 1):
            W1 = np.array([self.waypoints[n][0],self.waypoints[n][1]])
            W2 = np.array([self.waypoints[n+1][0],self.waypoints[n+1][1]])
            u0 = np.array([W2 - W1])
            norm = np.linalg.norm(u0)
            u0 = np.divide(u0, norm)
            x = np.array([0,0])
            u = np.array([1, 0])
            #wanted speed of the robot
            omf = 0.1
            #still don't know what this is
            dt = 0.1
            #or this
            Robot = np.array([[0, -1], [1, 0]])
            while not atThePoint:
                x = x + dt * u * omf
                xf = W2 + (omf - np.transpose(u0) * (W2 - x)) * u0
                v = xf - x
                v = v / np.linalg.norm(v)
                omd = (u[0] * v[1] - u[1] * v[0]) * 2
                R = Robot * dt
                ru = np.array([R[0][0] * u[0] + R[0][1] * u[1], R[1][0] * u[0] + R[1][1] * u[1]])
                ru = ru * omd
                u = u + ru
                u = u / np.norm(u)
                self.robot_drive(u)
                #If the robot's position is within 10 centimeters from the goal, we move on
                if inCircle(self.waypoints[n+1], 100, x):
                    atThePoint = True
