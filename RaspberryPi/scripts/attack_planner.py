#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import Float32 as Float

import math
import time

cmd_vel_pub = None
odom = Pose2D()
target = None

def attack_plnr():
	global cmd_vel_pub
	global odom
	global target

	print('Starting attack planner')

	rospy.init_node('attack_planner', anonymous=True)
	print('Created node')
	sub = rospy.Subscriber('target', Pose2D, callback)
	sub_odom = rospy.Subscriber('odometry', Pose2D, odom_callback)
	cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	arm_angle_pub = rospy.Publisher('arm_angle', Float, queue_size=10)
	rate = rospy.Rate(5)

	last_arm_command = 0
	while not rospy.is_shutdown():
		#rate.sleep()
		# print('LOOPING')
		time.sleep(0.5)
	   	# rospy.spin()
		cmd_vel = Twist()
		cmd_vel.angular.z = target.theta / 43.0;
		cmd_vel.linear.x = 0.25
		cmd_vel_pub.publish(cmd_vel)
		'''
		if odom == None or target == None:
			print('STUCK', (odom, target))
			continue
		distance = math.sqrt((target.x - odom.x)**2 + (target.y - odom.y)**2)
		dtheta = math.atan2((target.y - odom.y), (target.x - odom.x)) - odom.theta
		while dtheta > 2 * math.pi:
			dtheta -= 2 * math.pi
		while dtheta < -2 * math.pi:
			dtheta += 2 * math.pi
		print("dtheta: " , dtheta)
		
		cmd_vel = Twist()
		cmd_vel.linear.x = min(distance, 0.25)
		cmd_vel.angular.z = dtheta
		cmd_vel_pub.publish(cmd_vel)
		# print('Finished loop', cmd_vel)
		
		if (rospy.get_time() - last_arm_command) > 1:
			msg = Float()
			if distance < 0.17:
				msg.data = 30
			else:
				msg.data = 0 
			arm_angle_pub.publish(msg)
			last_arm_command = rospy.get_time()
		'''


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
