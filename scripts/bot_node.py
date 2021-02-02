#!/usr/bin/env python3

'''
This node helps the bot to `roam` in the mall without any specific 
heading using the lidar data. And it takes in camera feed from the 
camera mounted on the bot and uses it to estimate poses of different 
persons in the frame and then extracts face from it and then classifies
whether a specific face has a mask on it or not.
'''

import rospy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

import cv2
import numpy as np
from pose_detect import get_pose

SAVE = False
BASE_DIR = "/maskon_ws/src/maskon_pkg/"


global COUNT, OBS_DISTANCE, OBS_RAY_COUNT, LINEAR_VEL, ANGULAR_VEL, CHECK_EVERY
LINEAR_VEL = 0.22
ANGULAR_VEL = 0.8
OBS_DISTANCE_THRESH = 1.9
OBS_RAY_COUNT = 15
COUNT = 0
CHECK_EVERY = 10

def scan_callback(msg):

    right = msg.ranges[45+180:45+270]
    front = msg.ranges[-45:] + msg.ranges[:45]
    left = msg.ranges[45:45+90]

    obs = []
    c = 0
    for el in right:
        if el < OBS_DISTANCE_THRESH:
            c += 1
        if c >= OBS_RAY_COUNT:
            obs.append("RIGHT")
            break
    c = 0
    for el in left:
        if el < OBS_DISTANCE_THRESH:
            c += 1
        if c >= OBS_RAY_COUNT:
            obs.append("LEFT")
            break
    c = 0
    for el in front:
        if el < OBS_DISTANCE_THRESH:
            c += 1
        if c >= OBS_RAY_COUNT:
            obs.append("FRONT")
            break

    twist = Twist()

    twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0

    if "FRONT" not in obs:
        twist.linear.x = LINEAR_VEL
        # print("MOVING FRONT")
    elif "RIGHT" not in obs:
        twist.linear.x = LINEAR_VEL / 10
        twist.angular.z = -ANGULAR_VEL
        # print("MOVING RIGHT")
    else:
        twist.linear.x = LINEAR_VEL / 10
        twist.angular.z = ANGULAR_VEL
        # print("MOVING LEFT")
        
    pub.publish(twist)


def image_callback(msg):
    global COUNT

    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')[:, :, ::-1]

    if not COUNT % CHECK_EVERY:
        image = get_pose(cv_image)
        twist = Twist()

        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)
    else:
        image = cv_image[:, :, ::-1]
    COUNT += 1
    cv2.imshow("Bot's View", image)
    cv2.waitKey(1)

    if SAVE:
        cv2.imwrite(BASE_DIR + f"/images/run_2/{COUNT}.jpg", image)



if __name__ == '__main__':
    try:
        rospy.init_node('bot_node')
        
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        rospy.Subscriber("/rrbot/camera1/image_raw/", Image, image_callback)
        rospy.Subscriber("/scan", LaserScan, scan_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupted.")