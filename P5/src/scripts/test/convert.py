#!/usr/bin/python
import rospy
from std_msgs.msg import Int32
#import std_msgs.msg

pub = rospy.Publisher('chatter', Int32, queue_size=10)
rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(10) # 10hz

while not rospy.is_shutdown():
    
    abc = "h"
    #abchex = hex(int((abc), 16))
    #abchex = hex(ord(abc))
    #abchex = int(hex(abc), 16)
    #abchex = int(abc, 16)
    abchex = hex(abc)
    #abchex = struct.unpack("h", abc)
    rospy.loginfo(abchex)
    pub.publish(abchex)
    rate.sleep()
    
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
