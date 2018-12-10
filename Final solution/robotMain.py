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
end = (20, 75)

if __name__ == '__main__':
    imu = IMU()
    listener()
    graph = AStarGraph()
    while not rospy.is_shutdown():
        if start and end:
            if abs(start[0]) <= 200 and abs(start[1]) <= 200:
                #print('goal read:', end)
                result, cost = AStarSearch(start, end, graph)
                result = optimizeWaypoints(result)
                path = PlanThePath(result, imu)
                #print(imu.getHeading())
                while not path.nearTheGoal(end, 10, start):
                    path.move(start)
            else: print('The robot is out of bounds')
