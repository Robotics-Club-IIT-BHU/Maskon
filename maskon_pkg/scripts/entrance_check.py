#!/usr/bin/env python3

'''
This node takes in camera feed from a camera at the entrance of 
the mall and processes the image to extract faces and then classifies
whether a specific face has a mask on it or not.
'''

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import tensorflow as tf
import cv2 
import numpy as np
from mtcnn import MTCNN

SAVE = False
BASE_DIR = "/maskon_ws/src/maskon_pkg/"

def predict(image):

    image = cv2.resize(image, (image.shape[1] // 2, 
                                image.shape[0] // 2))

    faces = detector.detect_faces(image)

    boxes = []

    for face in faces:
        
        box = face['box']
        boxes.append(box)

        cropped_image = image[max(box[1] - 10, 0) :10 + box[1] + max(box[3], box[2]), max(0, box[0] - 10):10 + box[0] + max(box[3], box[2]), :]

        preds = np.squeeze(model.predict(np.array([cv2.resize(cropped_image, (64, 64)) / 255.0])))
   
        if preds[0] <= preds[1]:
            cv2.rectangle(image, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]), (255, 10, 10), 2)
            # cv2.putText(image, 'NO MASK', (box[0] + 10, box[1] + box[3]), cv2.FONT_HERSHEY_PLAIN, 1, (10, 10, 255), 1)
            rospy.loginfo('=== ALERT!! ===')
        else:
            cv2.rectangle(image, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]), (46, 204, 113), 2)
            # cv2.putText(image, 'MASK', (box[0], box[1] + box[3] + 10), cv2.FONT_HERSHEY_PLAIN, 1, (137, 165, 23), 1)

    return image, boxes


def image_callback(msg):

    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    image, _ = predict(cv_image)

    cv2.imshow('Image', image[:, :, ::-1])
    cv2.waitKey(1)

    if SAVE:
        cv2.imwrite(BASE_DIR + "/images/result_entrance.jpg", image[:, :, ::-1])
    


if __name__ == '__main__':
    try:
        rospy.init_node('entrance_check')
        
        detector = MTCNN()
        model = tf.keras.models.load_model(BASE_DIR + 'models/model-2.h5')

        rospy.Subscriber("/wall_camera/image_raw_wall", Image, image_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupted.")