#!/usr/bin/env python

"""
ROS Node for that acts as a PI compensator for the cmd_vel
"""

from model import ROBOT_D, ROBOT_R

import rospy
from geometry_msgs.msg import Pose2D, Vector3, Twist

cmd_vel_comp_pub = None

motor_angles_prev = Vector3()
time_prev = 0.

target_cmd_vel = None

def motor_angles_cb(motor_angles):
    global cmd_vel_comp_pub
    global motor_angles_prev
    global time_prev
    global target_cmd_vel

    if target_cmd_vel == None:
        return

    time_current = rospy.get_time()
    d_time = time_current - time_prev

    d_motor_right = motor_angles.x - motor_angles_prev.x
    d_motor_left = motor_angles.y - motor_angles_prev.y

    target_motor_right_speed = ((2 * target_cmd_vel.linear.x) - (target_cmd_vel.angular.z * ROBOT_D)) / (2 * ROBOT_R)
    target_motor_left_speed = ((2 * target_cmd_vel.linear.x) + (target_cmd_vel.angular.z * ROBOT_D)) / (2 * ROBOT_R)

    real_motor_right_speed = d_motor_right / d_time
    real_motor_left_speed = d_motor_left / d_time

    right_err = target_motor_right_speed - real_motor_right_speed
    left_err = target_motor_left_speed - real_motor_left_speed

    comp_right = 1 * right_err
    comp_left = 1 * left_err

    comp_cmd_vel = Twist()
    comp_cmd_vel.linear.x = (comp_left * ROBOT_R + comp_right * ROBOT_D) / 2
    comp_cmd_vel.angular.z = (comp_right * ROBOT_D - comp_left * ROBOT_R) / ROBOT_D
    cmd_vel_comp_pub.publish(comp_cmd_vel)

    time_prev = time_current
    motor_angles_prev = motor_angles
    

def cmd_vel_cb(cmd_vel):
    global target_cmd_vel
    target_cmd_vel = cmd_vel

def cmd_vel_compensator():
    global cmd_vel_comp_pub

    cmd_vel_comp_pub = rospy.Publisher('cmd_vel_comp', Twist, queue_size=10)
    rospy.init_node('cmd_vel_compensator')

    rospy.Subscriber('motor_angles', Vector3, motor_angles_cb)
    rospy.Subscriber('cmd_vel', Twist, cmd_vel_cb)

    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == "__main__":
    try:
        cmd_vel_compensator()
    except rospy.ROSInterruptException:
        pass
