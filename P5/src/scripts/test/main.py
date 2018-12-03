#!/usr/bin/python
import rospy
from notmain import *
from std_msgs.msg import Float32
#import std_msgs.msg

pub = rospy.Publisher('chatter', Float32 queue_size=10)
rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(10) # 10hz

while not rospy.is_shutdown():
    
    x = numbers(0)
    x.numberss()
    #String(data = int)
    rospy.loginfo(x.one)
    pub.publish(x.one)
    rate.sleep()
    
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

