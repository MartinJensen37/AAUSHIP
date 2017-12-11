#!/usr/bin/env python
import roslib
import rospy
import time
import std_msgs.msg

rospy.init_node("Publish_zeroes")

pub1 = rospy.Publisher("/pid_x/setpoint", std_msgs.msg.Float64, queue_size=1)
pub2 = rospy.Publisher("/pid_y/setpoint", std_msgs.msg.Float64, queue_size=1)
pub3 = rospy.Publisher("/pid_z/setpoint", std_msgs.msg.Float64, queue_size=1)

while(1) : 
	time.sleep(0.1)

	msg = std_msgs.msg.Float64()

	msg.data = 0

	pub1.publish(msg)
	pub2.publish(msg)
	pub3.publish(msg)
