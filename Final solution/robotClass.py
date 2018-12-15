#!/usr/bin/python
from __future__ import print_function
from __future__ import division
from di_sensors.inertial_measurement_unit import InertialMeasurementUnit
import math
import time
import numpy as np
import matplotlib.pyplot as plt
import sys
from easygopigo3 import EasyGoPiGo3

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
        #print(current)
    raise RuntimeError("A* failed to find a solution")
 
class IMU:

    def __init__(self):
        self.imu = InertialMeasurementUnit(bus = "GPG3_AD1") #RPI_1 GPG3_AD1

    def getHeadingDeg(self):
        #Read the magnetometer, gyroscope, accelerometer in rad
        print('IMU getting the data ...')
        mag   = self.imu.read_magnetometer()
        gyro  = self.imu.read_gyroscope()
        accel = self.imu.read_accelerometer()


        """pitch = np.arcsin(-accel[1] / np.linalg.norm(accel))
        roll = np.arcsin(accel[0] / n.linalg.norm(accel))
        y = -mag[0] * np.cos(roll) + mag[2] * np.sin(roll)
        x = mag[0] * np.sin(pitch) * np.sin(roll) + mag[1] * cos(pitch) + mag[2] * np.sin(pitch) * np.cos(roll)
        azimuth = np.arctan2(y, x)
        orientation = np.array([azimuth, pitch])"""

        angle = (180/math.pi * math.atan2(mag[0], mag[2]) % 360)
        #print('angle = ', angle)
        #print('angle = ', angle)
        #print('orientation = ', orientation)
        print('returning the angle')
        return angle #,angle
    
    def getHeading(self):
        angle = self.getHeadingDeg()
        angle = np.deg2rad(angle)
        orientation = np.array([np.cos(angle), np.sin(angle)])
        return orientation

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
        #Basically just means it can move between waypoints like a king in chess. Movement in every direction but only one waypoint at a time
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
                return 1000 #Extremely high cost to enter barrier squares
        return 1 #Normal movement cost

class PlanThePath:
#Add functions to find the next point, calculate needed values
   
    def __init__(self, path, imu):
        self.waypoints = path
        self.imu = imu

    def robot_drive(self, omf, uangle):
        gpg = EasyGoPiGo3()
        gpg.set_speed(180)
        #drive = gpg.forward()
        orientationangle = self.imu.getHeadingDeg()
        print("orientationangle = ", orientationangle)
        n = 0
        rightAngle = False
        #print(n)
        if omf < 0:
            rightAngle = True
        while not rightAngle:
            if uangle - 10 <= orientationangle and uangle + 10 >= orientationangle:
                rightAngle = True
                #print("1 orientationangle = ", orientationangle, 'uangle = ', uangle)
                #print("same though")
            else:
                temp = 0
                temp2 = 0
                for i in range(5, 359):
                    #temp += 1
                    #tempCurrent
                    #print('i', i)
                    #print(int(orientationangle), int(uangle))
                    if int(orientationangle) + i > 360:
                        if (int(orientationangle) + i) - 360 == int(uangle):
                            temp = i
                            #print('temp assigned: special', temp, i)
                    if int(orientationangle) + i == int(uangle):
                        temp = i
                        #print('temp assigned:', temp, i)
                    if int(orientationangle) - i < 0:
                        if 360 + (int(orientationangle) - i) == int(uangle):
                            temp2 = i
                    if int(orientationangle) - i == int(uangle):
                        temp2 = i
                        #print('temp2 assigned:', temp2, i)
                    if temp != 0 and temp2 != 0:
                        #print('breaks at', i)
                        break
                    #print('1:', temp, '2:', temp2)
                if temp2 < temp:
                    gpg.turn_degrees(-1 * temp2)
                    #print("2 orientationangle = ", orientationangle, 'uangle = ', uangle, 'diff cc=', temp2)
                else:
                    gpg.turn_degrees(temp)
                    #print("2 orientationangle = ", orientationangle, 'uangle = ', uangle, 'diff cw=', temp)
            orientationangle = self.imu.getHeadingDeg()
        if omf > 0:
            gpg.forward()
            print('moved')
        if omf < 0:
            gpg.stop()
            print('stopped')
                #print("3 orientationangle = ", orientationangle, 'uangle = ', uangle, 'diff =', diff_head)

                #drive
                #print("not same more", orientationangle)

    def nearTheGoal(self, center, radius, point):
        if (abs(point[0]) - abs(center[0]))**2 + (abs(point[1]) - abs(center[1]))**2 < radius**2:
            return True
        return False