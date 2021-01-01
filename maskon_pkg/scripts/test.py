#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_callback(msg):

    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    cv2.imshow('Image', cv2.resize(cv_image[:, :, ::-1], (960, 540)))
    cv2.waitKey(5)
    cv2.imwrite('result.jpg', cv_image[:, :, ::-1])




if __name__ == '__main__':
    try:
        rospy.init_node('testing_node')
        rospy.Subscriber("/wall_camera/image_raw_wall", Image, image_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test Interrupted.")