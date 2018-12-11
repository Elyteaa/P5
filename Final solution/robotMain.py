#!/usr/bin/python

import math
import numpy as np
import time #may be not needed
from robotClass import *
import rospy
from std_msgs.msg import Int32MultiArray

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
    else:
        return waypoints

def callback(data):
    global start
    start = data.data

def callback2(data):
    global end
    end = data.data

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("position", Int32MultiArray, callback)

    rospy.Subscriber("goal", Int32MultiArray, callback2)
    # spin() simply keeps python from exiting until this node is stopped
    #spin() fucks everything up and is not necesssary in rospy !
    #rospy.spin()

start = None
end = (50, 50)

if __name__ == '__main__':
    imu = IMU()
    listener()
    graph = AStarGraph()
    while not rospy.is_shutdown():
        if start and end:
            if abs(start[0]) <= 200 and abs(start[1]) <= 200:
                print('goal read:', end)
                result, cost = AStarSearch(start, end, graph)
                result = optimizeWaypoints(result)
                path = PlanThePath(result, imu)
                #print(imu.getHeading())

                R = np.array([[0, -1], [1, 0]])
                dt = 0.1 #resolution
                vel = 100

                n = 0
                while n <= len(path.waypoints)-2:
                    W1 = np.array([path.waypoints[n][0],path.waypoints[n][1]])
                    W2 = np.array([path.waypoints[n+1][0],path.waypoints[n+1][1]])
                    u0 = np.array([W2 - W1])
                    norm = np.linalg.norm(u0)
                    u0 = np.divide(u0, norm)
                    x = np.array([start[0], start[1]])
                    u = path.imu.getHeading()

                    while not path.nearTheGoal(W2, 5, start):
                        x = x + dt * u * vel
                        print('x=', x)
                        xf = W2 + (vel - np.transpose(u0) * (W2 - x)) * u0
                        v = xf - x
                        v = v / np.linalg.norm(v)
                        omd = (u[0] * v[1] - u[1] * v[0]) * 2
                        R = R * dt
                        ru = np.array([R[0][0] * u[0] + R[0][1] * u[1], R[1][0] * u[0] + R[1][1] * u[1]])
                        ru = ru * omd
                        u = u + ru
                        u = u / np.linalg.norm(u)
                        print('u=',u)
                        #uangle = np.array([math.sqrt(u[0]**2 +, np.sin(u)])
                        uangle = (180/math.pi * np.arctan2(u[1], u[0]) % 360)
                        #print("uangle = ", uangle)
                        path.robot_drive(vel, uangle)
                    print(n, 'out of', len(path.waypoints))
                    n += 1
            else: print('The robot is out of bounds')

