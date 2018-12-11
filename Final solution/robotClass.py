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
        print(current)
    raise RuntimeError("A* failed to find a solution")
 
class IMU:

    def __init__(self):
        self.imu = InertialMeasurementUnit(bus = "GPG3_AD1") #RPI_1 GPG3_AD1

    def getHeadingDeg(self):
        #Read the magnetometer, gyroscope, accelerometer in rad
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
   
    def __init__(self, path, imu):
        self.waypoints = path
        self.imu = imu

    def robot_drive(self, omf, uangle):
        gpg = EasyGoPiGo3()
        gpg.set_speed(omf)
        #drive = gpg.forward()
        orientationangle = self.imu.getHeadingDeg()
        #print("orientationangle = ", orientationangle)
        n = 0
        #print(n)
        while n < 1:
            if omf < 0:
                gpg.stop()
                break
            if uangle - 10 <= orientationangle and uangle + 10 >= orientationangle:
                if omf > 0:
                    #gpg.drive_cm(20)
                    gpg.forward()
                #print("1 orientationangle = ", orientationangle, 'uangle = ', uangle)
                #print("same though")
            else:
                temp = 0
                temp2 = 0
                for i in range(5, 359):
                    #temp += 1
                    #tempCurrent
                    if orientationangle + i == int(uangle):
                        temp = i
                    if orientationangle - i == int(uangle):
                        temp2 = i
                    if temp != 0 and temp2 != 0:
                        break
                if temp2 < temp:
                    gpg.turn_degrees(-1 * temp2)
                    print("2 orientationangle = ", orientationangle, 'uangle = ', uangle, 'diff cc=', temp2)
                else:
                    gpg.turn_degrees(temp)
                    print("2 orientationangle = ", orientationangle, 'uangle = ', uangle, 'diff cw=', temp)
                if omf > 0:
                    gpg.forward()

            """elif uangle - 10 < orientationangle:
                diff_head = orientationangle - uangle
                gpg.turn_degrees(-diff_head)
                if omf > 0:
                    #orientationangle = orientationangle + abs(diff_head)
                    #gpg.drive_cm(20, True)
                    gpg.forward()
                    #print("2 orientationangle = ", orientationangle, 'uangle = ', uangle, 'diff =', diff_head)

                    #drive
                    #print("not same less", orientationangle)
            elif uangle + 10 > orientationangle:
                diff_head = orientationangle - uangle

                #orientationangle = orientationangle - abs(diff_head)
                gpg.turn_degrees((-1)*diff_head)
                if omf > 0:
                    #gpg.drive_cm(2, True)
                    gpg.forward()"""
                #print("3 orientationangle = ", orientationangle, 'uangle = ', uangle, 'diff =', diff_head)

                #drive
                #print("not same more", orientationangle)
            n += 1

    def nearTheGoal(self, center, radius, point):
        if (point[0] - center[0])**2 + (point[1] - center[1])**2 < radius**2:
            return True
        return False

    """def move(self, current):
        Robot = np.array([[0, -1], [1, 0]])
        #atThePoint = False
        dt = 0.1
        omf = 100

        if self.n <= len(self.waypoints)-2:
            W1 = np.array([self.waypoints[self.n][0],self.waypoints[self.n][1]])
            W2 = np.array([self.waypoints[self.n+1][0],self.waypoints[self.n+1][1]])
            u0 = np.array([W2 - W1])
            norm = np.linalg.norm(u0)
            u0 = np.divide(u0, norm)
            x = np.array([current[0], current[1]])
            u = self.imu.getHeading()

            while not self.nearTheGoal(W2, 5, x):
                x = x + dt * u * omf
                print('x=',x)
                xf = W2 + (omf - np.transpose(u0) * (W2 - x)) * u0
                v = xf - x
                v = v / np.linalg.norm(v)
                omd = (u[0] * v[1] - u[1] * v[0]) * 2
                R = Robot * dt
                ru = np.array([R[0][0] * u[0] + R[0][1] * u[1], R[1][0] * u[0] + R[1][1] * u[1]])
                ru = ru * omd
                u = u + ru
                u = u / np.linalg.norm(u)
                print('u=',u)
                #uangle = np.array([math.sqrt(u[0]**2 +, np.sin(u)])
                uangle = np.arctan2(u[1], u[0])
                #print("uangle = ", uangle)
                self.robot_drive(u, omf, uangle)
            self.n += 1"""
