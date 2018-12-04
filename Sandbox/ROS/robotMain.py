#!/usr/bin/python
import rospy
import math
import numpy as np
import time #may be not needed
from robotClass import *
from std_msgs.msg import Float32MultiArray

#positions = []
class Listener():

    def __init__(self):
        self.position = []
        self.sub = rospy.Subscriber("position", Float32MultiArray, self.callback)

    def callback(data):
        rospy.loginfo("I receive %s", data.data)
        self.position = data.data

    #def start(self):
    #   self

#def callback(data):
 #   rospy.loginfo("I receive %s", data.data)
 #   positions = data.data
    #print(positions)
    
#def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #rospy.init_node('listener', anonymous=True)

    #rospy.Subscriber("position", Float32MultiArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()
    
if __name__ == '__main__':
    #while True:
    print('yes')
    rospy.init_node('listener', anonymous=True)
    my_node = Listener()
    #listener()
    #rospy.Subscriber("position", Float32MultiArray, callback)
    rospy.spin()
    print("position", positions)
    map1 = WayPoints() #the object that stores all way points of a map

    map1.addWayPoint(1, 0)
    map1.addWayPoint(0, 1)

    #print(map1.wayPoint[0])

    path = PlanThePath(map1.wayPoint[0], map1.wayPoint[1])
    #print(path.W1, path.W2)
    path.plan()
    rospy.spin()
