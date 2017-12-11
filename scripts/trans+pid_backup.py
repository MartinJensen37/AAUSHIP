#!/usr/bin/env python
import rospy
import time
import geometry_msgs.msg
#from aauship_control.msg import *
import tf
import math
import numpy as np
import std_msgs.msg

rospy.init_node('range_proportional_control') #initialise so we can publish and subscribe

listener = tf.TransformListener() #create a listener, not a subcriber
#pub = rospy.Publisher('lli_input', LLIinput, queue_size=1000) #LLI publisher for the thrust values

pubx   = rospy.Publisher('/pid_x/state'  , std_msgs.msg.Float64, queue_size=1)
puby   = rospy.Publisher('/pid_y/state'  , std_msgs.msg.Float64, queue_size=1)
pubyaw = rospy.Publisher('/pid_yaw/state', std_msgs.msg.Float64, queue_size=1)


#pub_msg = LLIinput() #initialize the message container
#pub_msg.DevID = int(10)
#pub_msg.MsgID = int(5)

time.sleep(1) #sleep to let transforms arrive before we start polling them

xdn = 1
ydn = 0
zdn = 0
while(1):
	time.sleep(0.1)
        (trans,rot) = listener.lookupTransform('/usb_cam', '/map', rospy.Time(0)) #get the transform between map and usb_cam
	euler = tf.transformations.euler_from_quaternion(rot)
	yaw = euler[1]
	x = trans[2]
	y = trans[0]

	#Transformation matrix from b to n
	Tn = np.array([[math.cos(yaw), -math.sin(yaw), x],[ math.sin(yaw), math.cos(yaw), y],[ 0, 0, 1]])

	#Vector with the desired postions of the boat in NED frame
	Pd = np.array([xdn, ydn, 1])
	# The inverse of Tb (from n to b)
	Tinv = np.linalg.inv(Tn)

	#Vector with desired postions in boat frame
	Tb = Tinv.dot(Pd)
	
	xdb  =std_msgs.msg.Float64()
	ydb  =std_msgs.msg.Float64()
	yawdb=std_msgs.msg.Float64()

	#Output of the transformation matrix
	xdb.data = Tb[0]

	ydb.data = Tb[1]

	yawdb.data = zdn - yaw

	pubx.publish(    xdb)
        puby.publish(    ydb)
        pubyaw.publish(yawdb)

	
	#Publish the vector containing the desired positions to the topic
	#pub_msg.Data = Th

	print ("x ", xdb, " y ", ydb, " yaw ", yawdb)

	#pub.publish(pub_msg)
