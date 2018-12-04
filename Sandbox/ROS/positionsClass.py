#!/usr/bin/python
import rospy
from std_msgs.msg import Float32MultiArray

class Publisher(object):

	def __init__(self):
		self.x1 = 1.5
		self.a = None
		self.loop_rate = rospy.Rate(10)
		self.pub = rospy.Publisher('position', Float32MultiArray, queue_size=10)

	def callback(self, msg):
		self.a = msg.data
		self.x1 = self.a + 0.5
		rospy.loginfo("1x: {}".format(self.x1))

	def start(self):
		rospy.loginfo("In attesa")
		while not rospy.is_shutdown():
			for ii in range (1,10):
				self.pub.publish(self.x1)
				self.loop_rate.sleep()


class CPRMeasurement:

	def __init__(self, CPRID, ultrasoundLevel, numMasters, timeDifference, timestampMS, transmitterID, masterID, RSSI):
		self.CPRID = CPRID
		self.ultrasoundLevel = ultrasoundLevel
		self.numMasters = numMasters
		self.timeDifference = timeDifference
		self.timestampMS = timestampMS
		self.transmitterID = transmitterID
		self.masterID = masterID
		self.RSSI = RSSI


class trueMeasurement: #temperature is needed here
	
	#data of incomngMeasurement
	def __init__(self, transmitterID, masterID, RSSI, ultrasoundLevel, timestampMS, CPRID, timeDifference, temperature):
		self.timestampMS = timestampMS
		self.receiverID = CPRID
		self.ultrasoundLevel = ultrasoundLevel
		self.distance = (timeDifference * (331.4+0.6*temperature)/1000) + 18
		self.transmitterID = transmitterID
		self.RSSI = RSSI
		self.masterID = masterID
