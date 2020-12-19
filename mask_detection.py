import tensorflow as tf
import cv2
import numpy as np
from mtcnn import MTCNN
import os

detector = MTCNN()
cap = cv2.VideoCapture(0)
# Load model ## Change filename here
model = tf.keras.models.load_model('model-2.h5')

while cap.isOpened():

    _, image = cap.read()
    img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    faces = detector.detect_faces(cv2.resize(img, (image.shape[1] // 4, image.shape[0] // 4)))

    for face in faces:
        box = face['box']

        box = [el * 4 for el in box]
        cropped_image = image[max(box[1] - 10, 0) :10 + box[1] + max(box[3], box[2]), max(0, box[0] - 10):10 + box[0] + max(box[3], box[2]), :]

        # Resize it according to your model
        preds = np.squeeze(model.predict(np.array([cv2.resize(cropped_image, (64, 64)) / 255.0])))

        print(preds)
        if preds[0] <= preds[1]:
            cv2.rectangle(image, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]), (10, 10, 255), 2)
            cv2.putText(image, 'NO MASK', (box[0] + 10, box[1] + box[3]), cv2.FONT_HERSHEY_PLAIN, 1, (10, 10, 255), 1)
        else:
            cv2.rectangle(image, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]), (193, 134, 46 ), 2)
            cv2.putText(image, 'MASK', (box[0], box[1] + box[3] + 10), cv2.FONT_HERSHEY_PLAIN, 1, (193, 134, 46), 1)

    cv2.imshow('Image', image)
    cv2.waitKey(1)

