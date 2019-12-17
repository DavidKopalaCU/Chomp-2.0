#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Pose2D

import math

cmd_vel_pub = None
odom = None
target = None

def attack_plnr():
	global cmd_vel_pub
	global odom
	global target

	rospy.init_node('attack_planner', anonymous=True)
	sub = rospy.Subscriber('target', Pose2D, callback)
	sub_odom = rospy.Subscriber('odometry', Twist, odom_callback)
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	rate = rospy.Rate(5)
	while not rospy.is_shutdown():
		rate.sleep()
	   	rospy.spin()
		if odom == None or target == None:
			continue
		dx = math.sqrt((target.x - odom.x)**2 + (target.y - odom.y)**2)
		dtheta = math.atan((target.y - odom.y) / (target.x - odom.x)) - odom.theta
		
		cmd_vel = Twist()
		cmd_vel.linear.x = max(dx, 0.5)
		cmd_vel.angular.z = dtheta
		cmd_vel_pub.publish(cmd_vel)

def odom_callback(data):
	global odom
	odom = data


def callback(data):
	global target
	target = data

if __name__ == "__main__":
    try:
	attack_plnr()
    except rospy.ROSInterruptException:
	pass
