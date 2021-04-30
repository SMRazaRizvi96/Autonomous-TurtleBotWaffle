#!/usr/bin/env python

# Python libs
import sys
import time
import math
import random
import actionlib
import subprocess
import signal

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

import imutils

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy
import smach
import smach_ros

# Ros Messages
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalID
from sofar_project.msg import Ball

black_ball = Ball()
red_ball = Ball()
yellow_ball = Ball()
green_ball = Ball()
blue_ball = Ball()
magenta_ball = Ball()

VERBOSE = False

def clbk_laser(msg):
    global regions_

    regions_ = {
	'front_right': min(min(msg.ranges[45:89]), 10),
	'front_left': min(min(msg.ranges[90:134]), 10),
    }


def robotPos(currentPos):
    
    global robot_x,robot_y
    robot_x = currentPos.pose.pose.position.x
    robot_y = currentPos.pose.pose.position.y


def imageCallback(image):
	
    global np_arr
    if VERBOSE:
       print ('received image of type: "%s"' % image.format)

    #### direct conversion to CV2 ####
    np_arr = np.fromstring(image.data, np.uint8)


def detectBall():

        """
	It uses Open CV to process the image received, and detects the contours of the different colored balls in the image using different masks for each color. 
	If any contour is found, it passes the radius and center of the detected countour, and the detected ball object to the function track(). track() is implemented for going near the ball autonomously.  
	"""
	
	global np_arr
	image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

	blackLower = (0, 0, 0) 
	blackUpper = (5,50,50)
	redLower = (0, 50, 50)
	redUpper = (5, 255, 255)
	yellowLower = (25, 50, 50) 
	yellowUpper = (35, 255, 255)
	greenLower = (50, 50, 50) 
	greenUpper = (70, 255, 255)
	blueLower = (100, 50, 50) 
	blueUpper = (130, 255, 255)
	magentaLower = (125, 50, 50) 
	magentaUpper = (150, 255, 255)

        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

	mask_blk = cv2.inRange(hsv, blackLower, blackUpper)
        mask_blk = cv2.erode(mask_blk, None, iterations=2)
        mask_blk = cv2.dilate(mask_blk, None, iterations=2)

	mask_r = cv2.inRange(hsv, redLower, redUpper)
        mask_r = cv2.erode(mask_r, None, iterations=2)
        mask_r = cv2.dilate(mask_r, None, iterations=2)

	mask_y = cv2.inRange(hsv, yellowLower, yellowUpper)
        mask_y = cv2.erode(mask_y, None, iterations=2)
        mask_y = cv2.dilate(mask_y, None, iterations=2)

	mask_g = cv2.inRange(hsv, greenLower, greenUpper)
        mask_g = cv2.erode(mask_g, None, iterations=2)
        mask_g = cv2.dilate(mask_g, None, iterations=2)

        mask_blu = cv2.inRange(hsv, blueLower, blueUpper)
        mask_blu = cv2.erode(mask_blu, None, iterations=2)
        mask_blu = cv2.dilate(mask_blu, None, iterations=2)

	mask_m = cv2.inRange(hsv, magentaLower, magentaUpper)
        mask_m = cv2.erode(mask_m, None, iterations=2)
        mask_m = cv2.dilate(mask_m, None, iterations=2)
        #cv2.imshow('mask', mask)

        cnts_blk = cv2.findContours(mask_blk.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
	cnts_r = cv2.findContours(mask_r.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
	cnts_y = cv2.findContours(mask_y.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
	cnts_g = cv2.findContours(mask_g.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
	cnts_blu = cv2.findContours(mask_blu.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
	cnts_m = cv2.findContours(mask_m.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)

        cnts_blk = imutils.grab_contours(cnts_blk)
	cnts_r = imutils.grab_contours(cnts_r)
	cnts_y = imutils.grab_contours(cnts_y)
	cnts_g = imutils.grab_contours(cnts_g)
	cnts_blu = imutils.grab_contours(cnts_blu)
	cnts_m = imutils.grab_contours(cnts_m)

        center = None
	c = 0
	radius = 0

	global black_ball, red_ball, yellow_ball, green_ball, blue_ball, magenta_ball

        # only proceed if at least one contour was found
        if len(cnts_blk) > 0 and black_ball.detected_flag != True:
            c = max(cnts_blk, key=cv2.contourArea)
	    ((x, y), radius) = cv2.minEnclosingCircle(c)
	    M = cv2.moments(c)
	    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
	    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
	    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
	    black_ball = track(radius, center, black_ball)
	    pub_blackBall.publish(black_ball)
	    print ("Black ball detected.\n")

	elif len(cnts_r) > 0 and red_ball.detected_flag != True:
            c = max(cnts_r, key=cv2.contourArea)
	    ((x, y), radius) = cv2.minEnclosingCircle(c)
	    M = cv2.moments(c)
	    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
	    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
	    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
	    red_ball = track(radius, center, red_ball)
	    pub_redBall.publish(red_ball)
	    print ("Red ball detected.\n")
	    
	elif len(cnts_y) > 0 and yellow_ball.detected_flag != True:
            c = max(cnts_y, key=cv2.contourArea)
	    ((x, y), radius) = cv2.minEnclosingCircle(c)
	    M = cv2.moments(c)
	    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
	    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
	    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
	    yellow_ball = track(radius, center, yellow_ball)
	    pub_yellowBall.publish(yellow_ball)
	    print ("Yellow ball detected.\n")

	elif len(cnts_g) > 0 and green_ball.detected_flag != True:
            c = max(cnts_g, key=cv2.contourArea)
	    ((x, y), radius) = cv2.minEnclosingCircle(c)
	    M = cv2.moments(c)
	    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
	    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
	    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
	    green_ball = track(radius, center, green_ball)
	    pub_greenBall.publish(green_ball)
	    print ("Green ball detected.\n")

	elif len(cnts_blu) > 0 and blue_ball.detected_flag != True:
            c = max(cnts_blu, key=cv2.contourArea)
	    ((x, y), radius) = cv2.minEnclosingCircle(c)
	    M = cv2.moments(c)
	    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
	    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
	    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
	    blue_ball = track(radius, center, blue_ball)
	    pub_blueBall.publish(blue_ball)
	    print ("Blue ball detected.\n")

	elif len(cnts_m) > 0 and magenta_ball.detected_flag != True:
            c = max(cnts_m, key=cv2.contourArea)
	    ((x, y), radius) = cv2.minEnclosingCircle(c)
	    M = cv2.moments(c)
	    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
	    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
	    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
	    magenta_ball = track(radius, center, magenta_ball)
	    pub_magentaBall.publish(magenta_ball)
	    print ("Magenta ball detected.\n")

	cv2.imshow('window', image_np)
	cv2.waitKey(2)
	c = 0

def track(radius, center, detected_ball):

	"""
	This function is used to associate the coordinate to every ball in a way that it records its own location as the ball location, once the robot has reached quite near to the ball. The robot approaches the ball while avoiding obstacles and when a certain threshold of the radius of the ball has been reached the robot stops.  
	"""

	global regions_
	vel = Twist()
	
	if (radius > 200):
		print 'Reached'
		vel.angular.z = 0.0
		vel.linear.x = 0.0
		pub_cmd_vel.publish(vel)
		detected_ball.detected_flag = True
		detected_ball.location.x = robot_x
		detected_ball.location.y = robot_y
		print('Ball x: ', detected_ball.location.x,' Ball y: ', detected_ball.location.y)

	elif (regions_['front_right'] > 0.5 and regions_['front_left'] > 0.5):
		vel.angular.z = -0.0005*(center[0]-960)
		vel.linear.x = -0.007*(radius-200)
		pub_cmd_vel.publish(vel)

	else:
		if (regions_['front_right'] < 0.5):
			vel.angular.z = -0.2
			vel.linear.x = 0.0			
			pub_cmd_vel.publish(vel)

		if (regions_['front_left'] < 0.5):
			vel.angular.z = -0.2
			vel.linear.x = 0.0
			pub_cmd_vel.publish(vel)

	return detected_ball


def main():

    """
    The main function initializes the ros node, subscribes to the topic: robot's odometry topic ("/odom"), laser scan data (/scan), and the robot's camera image topic ("/camera/rgb/image_raw/compressed"), and creates a publisher for the robot's velocity commands on the topic ("/cmd_vel"), and publishes the locations of the balls on their corresponding topics.

This entire .py file corresponds to the following three subsystems:
1) Rigid Body Detector
2) Robot Controller
3) Path Planner 
    """


    global pub_cmd_vel, pub_cancel, child, pub_blueBall, pub_blackBall, pub_redBall, pub_greenBall, pub_magentaBall, pub_yellowBall 
    sub_camera = rospy.Subscriber("/camera/rgb/image_raw/compressed",
                                           CompressedImage, imageCallback,  queue_size=1)
    sub_odom = rospy.Subscriber("/odom", Odometry, robotPos)
    sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    
    pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    
    pub_blueBall = rospy.Publisher("/blueBall_loc", Ball, queue_size = 1)
    pub_blackBall = rospy.Publisher("/blackBall_loc", Ball, queue_size = 1)
    pub_redBall = rospy.Publisher("/redBall_loc", Ball, queue_size = 1)
    pub_greenBall = rospy.Publisher("/greenBall_loc", Ball, queue_size = 1)
    pub_magentaBall = rospy.Publisher("/magentaBall_loc", Ball, queue_size = 1)
    pub_yellowBall = rospy.Publisher("/yellowBall_loc", Ball, queue_size = 1) 
    
    rospy.init_node('sofar_project')
    time.sleep(5)
    
    while(1):
    	detectBall()

    rospy.spin()


if __name__ == '__main__':
    main()






