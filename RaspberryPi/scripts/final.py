#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image as msg_Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, Pose2D
import sys
import os

import numpy as np
import cv2
import time
from math import sin, cos, radians

# from racecar_flexbe_states.msg import Twist_float 

class ImageListener:
    def __init__(self, topic):
	self.centroid = None
        self.topic = topic
        self.bridge = CvBridge()
	self.odom = Pose2D()
        self.sub = rospy.Subscriber(topic, msg_Image, self.imageDepthCallback)
        self.sub_depth = rospy.Subscriber('/camera/depth/image_rect_raw', msg_Image, self.get_distance)
	self.sub_odom = rospy.Subscriber('/odometry', Pose2D, self.odometryCallback)
	self.pub_target = rospy.Publisher('/target', Pose2D, queue_size=1)


    def odometryCallback(self, data):
	self.prev_odom = data

    def imageDepthCallback(self, data):
        try:
	    # we select bgr8 because its the OpenCv encoing by default
	    #start_time = time.time()
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)

	    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) # convert to HSV
    	    lower_red_1 = np.array([30, 150, 50])
   	    upper_red_1 = np.array([255, 255,180])
    	    #lower_red_2 = np.array([20,100,100])
   	    #upper_red_2 = np.array([30,255,255])
    	    mask_1 = cv2.inRange(hsv, lower_red_1, upper_red_1 )
    	    #mask_2 = cv2.inRange(hsv, lower_red_2, upper_red_2 )

	    #compute teh center of teh contour
            M = cv2.moments(mask_1)
            cX = int(M['m10'] / M['m00'])
            cY = int(M['m01'] / M['m00'])
	    height,width,channels = cv_image.shape
	    # print(height,width,channels)
	    self.centroid = (float(cX) / width, float(cY) / height)

	    cv2.circle(mask_1, (cX,cY), 10, 255, -1) 
        except CvBridgeError as e:
            print(e)

	except ZeroDivisionError as e:
	    print(e)	
	
	#print(self.follow_corridor.curr_steering_angle)
	# self.pub.publish(self.msg)
	#cv2.imshow("mask_1", mask_1)y	#cv2.imshow("mask_2", mask_2)
        #cv2.waitKey(1)


	#print( time.time() - start_time)



    def calculate_heading(self):
	pass
    def get_distance(self, data):
	if self.centroid == None:
	    return
        cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
	height,width = cv_image.shape
	# print(height,width)

	cx_scaled = int(width * self.centroid[0])
	cy_scaled = int(height * self.centroid[1])
	print(cx_scaled,cy_scaled,cv_image[cy_scaled,cx_scaled])

        # d_theta = 1 - cx_scaled / (1.0*width/2.0)
	# msg = Twist()
	# msg.angular.z = d_theta
	# self.pub.publish(msg)
	
	# Angle of the target in frame
	distance = cv_image[cy_scaled, cx_scaled] / 1000.0
	angle = (self.centroid[0] - 0.5) * 86
	dy = cos(radians(angle)) * distance
	dx = sin(radians(angle)) * distance
	target_position = Pose2D()
	target_position.x = dx + self.odom.x - 0.1 # OFFSET FROM CENTER
	target_position.y = dy + self.odom.y
	# target_position.theta = angle # VERBOSE INFO ONLY
	self.pub_target.publish(target_position)

	cv2.circle(cv_image, (cx_scaled,cy_scaled), 2, 255*255, -1)

	cv2.imshow("Depth", cv_image)
	cv2.waitKey(1)
	
	pass

    def pid(self):
	pass
   
    def ShapeDetector(self):
	pass

def main():
    rospy.init_node('image_listener', anonymous=True)
    topic = '/camera/color/image_raw'
    listener = ImageListener(topic)
    try:
        rospy.spin()
    except KeyboardInterrupt:
	print('Shutting down')
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
