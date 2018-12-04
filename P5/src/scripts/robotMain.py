#!/usr/bin/python

import math
import numpy as np
import time #may be not needed
from robotClass import *
import rospy
from std_msgs.msg import Float32MultiArray

def optimizeWaypoints(waypoints):
	if len(waypoints) > 2:
		for x in range(0,len(waypoints)-2):
			if isBetween(waypoints[x], waypoints[x+2], waypoints[x+1]):
				#print(waypoints[x], waypoints[x+2], waypoints[x+1])
				del waypoints[x+1]
				#print(waypoints)
				return optimizeWaypoints(waypoints)

def callback(data):
	graph = AStarGraph()
	start = data.data
	end = (700, 700)
	if abs(data.data[0]) <= 200 and abs(data.data[1]) <= 200:
		result, cost = AStarSearch(start, end, graph)
		result = optimizeWaypoints(result)
		path = PlanThePath(result)
		path.move()
	else: print('The robot is out of bounds')

def listener():
	# In ROS, nodes are uniquely named. If two nodes with the same
	# node are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rospy.init_node('listener', anonymous=True)

	rospy.Subscriber("position", Float32MultiArray, callback)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	listener()