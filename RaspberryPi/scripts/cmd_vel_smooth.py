#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

import time
from math import fabs

cmd_vel_sub = None
cmd_vel_smooth_pub = None

STEP_SIZE = 0.005
DELTA_TIME = 0.01

cmd_vel_current = Twist()
cmd_vel_target = None

def entry():
	global cmd_vel_sub
	global cmd_vel_smooth_pub

	global MAX_STEP_SIZE
	global DELTA_TIME

	global cmd_vel_current
	global cmd_vel_target

	rospy.init_node('cmd_vel_smooth', anonymous=True)
	cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)
	cmd_vel_smooth_pub = rospy.Publisher('cmd_vel_smooth', Twist, queue_size=10)

	while not rospy.is_shutdown():
		time.sleep(DELTA_TIME)
		if cmd_vel_target == None or cmd_vel_target == cmd_vel_current:
			# print('Continuing')
			continue
		d_x = cmd_vel_target.linear.x - cmd_vel_current.linear.x
		if d_x < 0:
			d_x = -min(STEP_SIZE, fabs(d_x))
		else:
			d_x = min(STEP_SIZE, d_x)
		cmd_vel_current.linear.x += d_x

		d_z = cmd_vel_target.angular.z - cmd_vel_current.angular.z
		if d_z < 0:
			d_z = -min(STEP_SIZE, fabs(d_z))
		else:
			d_z = min(STEP_SIZE, d_z)
		cmd_vel_current.angular.z += d_z
		
		cmd_vel_smooth_pub.publish(cmd_vel_current)
		# print('Published!')

def cmd_vel_callback(data):
	global cmd_vel_target
	cmd_vel_target = data

if __name__ == "__main__":
	try:
		entry()
	except rospy.ROSInterruptException:
		pass
