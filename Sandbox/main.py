#!/usr/bin/python
import rospy
from notmain import *
from std_msgs.msg import Int32
one = 0
pub = rospy.Publisher('chatter', Int32, queue_size=10)
rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(10) # 10hz

while not rospy.is_shutdown():
    
    x = numbers(one)
    #String(data = int)
    rospy.loginfo(x)
    pub.publish(x)
    rate.sleep()
    
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

