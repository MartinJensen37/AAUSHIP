#!/usr/bin/env python
import rospy
import time
import std_msgs.msg
import sys, select, termios, tty



def getKey():
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

rospy.init_node('Emergency_stop_node')

settings = termios.tcgetattr(sys.stdin)
stop = rospy.Publisher('/stop'  , std_msgs.msg.Float64, queue_size=1)


msg = std_msgs.msg.Float64()

while(1):
	key = getKey()
	if(key == 'b'):
		msg.data = 1
		stop.publish(msg)

	if(key == ' '):
		msg.data = 0
		stop.publish(msg)



