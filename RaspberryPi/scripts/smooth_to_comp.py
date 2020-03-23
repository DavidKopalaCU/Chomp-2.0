#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

cmd_vel_comp_pub = None

def entry():
	global cmd_vel_comp_pub

	rospy.init_node('cmd_vel_smooth_comp', anonymous=True)
	cmd_vel_smooth_sub = rospy.Subscriber('cmd_vel_smooth', Twist, cmd_vel_smooth_callback)
	cmd_vel_comp_pub = rospy.Publisher('cmd_vel_comp', Twist, queue_size=10)

	while not rospy.is_shutdown():
		pass

def cmd_vel_smooth_callback(data):
	global cmd_vel_comp_pub
	cmd_vel_comp_pub.publish(data)

if __name__ == "__main__":
	try:
		entry()
	except rospy.ROSInterruptException:
		pass
