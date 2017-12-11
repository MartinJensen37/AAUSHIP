#!/usr/bin/env python

import rospy
import time

import math
import numpy as np
import std_msgs.msg
from aauship_control.msg import *


x = 0
y = 0
yaw = 0
stop = 0

def xcb(data):
	global x
	x    = data.data
	#print ("callback", x)
def ycb(data):
	global y
	y    = data.data
def yawcb(data):
	global z
	yaw  = data.data

def stopcb(data):
	global stop
	stop = data.data
	

def send_lli(msgid, data, pub_):
	pub_msg = LLIinput()

	pub_msg.DevID = int(10)
	pub_msg.MsgID = int(msgid)
	pub_msg.Data = int(data)
	pub_msg.Time = 0
	pub_.publish(pub_msg)


rospy.init_node('keyboard_teleop_node')

subx   = rospy.Subscriber('/pid_x/control_effort'  , std_msgs.msg.Float64, xcb)
suby   = rospy.Subscriber('/pid_y/control_effort'  , std_msgs.msg.Float64, ycb)
subyaw = rospy.Subscriber('/pid_yaw/control_effort', std_msgs.msg.Float64, yawcb)
substop= rospy.Subscriber('/stop'                  , std_msgs.msg.Float64, stopcb)


pub    = rospy.Publisher( '/lli_input', LLIinput, queue_size=1000)

while (1):
	time.sleep(0.05)
	#print("main", x)
	f1_force  = -x + yaw
	f2_force  = -x - yaw
	fbt_force =  y + yaw
	rbt_force =  y - yaw

	
	if (f1_force > 0):
		f1_pwm = (f1_force * 6.6044 + 60)*stop
	else: 
		f1_pwm = (f1_force * 8.5 - 100)   *stop

        if (f2_force > 0):
                f2_pwm = (f2_force * 6.6044 + 60)*stop
        else:
                f2_pwm = (f2_force * 8.5 - 100)   *stop


	fbt_pwm = fbt_force * 100 * stop
	rbt_pwm = fbt_force * 100 * stop
	

	send_lli(5, f1_pwm, pub) 
	send_lli(3, f2_pwm, pub)
	send_lli(22, abs(fbt_pwm), pub)
	if (fbt_pwm < 0):
		send_lli(35, 0, pub)
	send_lli(21, abs(rbt_pwm), pub)
	if (rbt_pwm < 0):
		send_lli(34, 0, pub)


