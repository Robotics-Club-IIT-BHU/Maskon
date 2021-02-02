'''
Not a ROS Node just a helper module to detect poses in 
a given frame
'''

import tensorflow as tf
import cv2
import time
import os
import numpy as np
import posenet

model = 101
scale_factor = 1.0


def get_pose(image):
    global sess
        
    input_image, draw_image, output_scale = posenet.process_input(image, 
                                        scale_factor=scale_factor, 
                                        output_stride=output_stride)

    heatmaps_result, offsets_result, displacement_fwd_result, displacement_bwd_result = sess.run(
        model_outputs,
        feed_dict={'image:0': input_image}
    )

    pose_scores, keypoint_scores, keypoint_coords = posenet.decode_multiple_poses(
        heatmaps_result.squeeze(axis=0),
        offsets_result.squeeze(axis=0),
        displacement_fwd_result.squeeze(axis=0),
        displacement_bwd_result.squeeze(axis=0),
        output_stride=output_stride,
        max_pose_detections=10,
        min_pose_score=0.25)

    keypoint_coords *= output_scale

    # If you need to draw human poses
    # draw_image = posenet.draw_skel_and_kp(
    #         draw_image, pose_scores, keypoint_scores, keypoint_coords,
    #         min_pose_score=0.25, min_part_score=0.25)

    if keypoint_coords[0][0][0] == 0 and keypoint_coords[0][0][1] == 0:
        return image
    
    return get_face(image, keypoint_coords)
    
def get_face(image, keypoint_coords):

    leftEar = keypoint_coords[0][3]
    rightEar = keypoint_coords[0][4]

    leftEye = keypoint_coords[0][1]
    rightEye = keypoint_coords[0][2]

    nose = keypoint_coords[0][0]

    image = np.array(image)

    # Checking if the person is turned opposite side
    if keypoint_coords[0][9][1] < keypoint_coords[0][8][1]:
        return image

    pt1 = (int(rightEar[1]), int(rightEar[0]))

    w = abs(int(leftEar[1]) - int(rightEar[1]))

    offs = w // 2
    
    pt1 = (pt1[0], max(0, pt1[1] - offs))

    pt2 = (pt1[0] + w, pt1[1] + w)

    cv2.rectangle(image, pt1, pt2, (46, 204, 113), 2)

    return draw_bb(image, pt1, pt2)

def draw_bb(image, pt1, pt2):

    pad = (pt2[1] - pt1[1]) // 10
    cropped_image = image[max(pt1[1] - pad, 0):pt2[1] + pad, max(pt1[0] - pad, 0):pt2[0] + pad, :]

    preds = np.squeeze(model.predict(
        np.array([cv2.resize(cv2.cvtColor(cropped_image, cv2.COLOR_BGR2RGB), (64, 64)) / 255.0])))

    if preds[1] >= 0.92:
        cv2.rectangle(image, pt1, pt2, (10, 10, 255), 2)
        print("=== PERSON WITH NO MASK ===")
    else:
        cv2.rectangle(image, pt1, pt2, (46, 204, 113), 2)


    return image


global sess
sess = tf.compat.v1.Session() 

model_cfg, model_outputs = posenet.load_model(model, sess)
output_stride = model_cfg['output_stride']

model = tf.keras.models.load_model("/home/vishwas/Workspace/maskon_ws/src/maskon_pkg/models/model-2.h5")

# 0	nose
# 1	leftEye
# 2	rightEye
# 3	leftEar
# 4	rightEar