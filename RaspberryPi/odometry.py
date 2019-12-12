from model import ROBOT_D, ROBOT_R

import rospy
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Vector3

from math import sin
from math import cos

odom_pose = Pose2D()
time_prev = 0.
prev_angles = Vector3()

odometry_pub = None

def odometry_m():
    global odometry_pub
    
    odometry_pub = rospy.Publisher('odometry', Pose2D, queue_size = 10)
    rospy.init_node('odometry_node')
    
    rospy.Subscriber('motor_angles', Vector3, motor_angle_callback)

    while not rospy.is_shutdown():
        rospy.spin()


def motor_angle_callback(arg):
    global time_prev
    global odom_pose
    global prev_angles
    global odometry_pub
    
    current_time = rospy.get_time()
    d_time = current_time - time_prev
    
    d_motor_right = arg.x - prev_angles.x
    d_motor_left = arg.y - prev_angles.y
    
    d_x = (ROBOT_R * (d_motor_left + d_motor_right)) / 2
    d_theta = (ROBOT_R * (d_motor_right - d_motor_left)) / ROBOT_D
    
    odom_pose.theta += d_theta
    odom_pose.x += cos(odom_pose.theta) * d_x
    odom_pose.y += sin(odom_pose.theta) * d_x
    
    time_prev = current_time
    prev_angles = arg
    odometry_pub.publish(odom_pose)


if __name__ == "__main__":
    try:
        odometry_m()
    except rospy.ROSInterruptExcpetion:
        pass

