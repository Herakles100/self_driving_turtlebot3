#!/usr/bin/env python3
import os
import cv2
import numpy as np
import tensorflow as tf
from yolov3.yolov3_tf2.models import YoloV3Tiny
from yolov3.yolov3_tf2.dataset import transform_images
from yolov3.yolov3_tf2.utils import draw_outputs


def main():

    # Get absolute base path for the folder containing the scripts
    base_path = os.path.dirname(os.path.abspath(__file__))

    # Image to be detected
    img_raw = tf.image.decode_image(
        open(os.path.join(base_path, 'yolov3/data/stop_sign.jpg'), 'rb').read(), channels=3)

    # Number of classes can be detected using tiny-yolo
    yolo = YoloV3Tiny()

    # Load trained weights for tiny-yolo
    yolo.load_weights(os.path.join(base_path, 'yolov3/checkpoints/yolov3-tiny.tf')).expect_partial()

    # Load class names that can be detected
    class_names = [c.strip()
                   for c in open(os.path.join(base_path, 'yolov3/data/coco.names')).readlines()]

    # Preprocess the image
    img = tf.expand_dims(img_raw, 0)
    img = transform_images(img, 416)

    # Detect objects in the image
    boxes, scores, classes, nums = yolo(img)

    # Loop through the detections
    for i in range(nums[0]):
        # Only show the detection result of stop sign
        if 'stop sign' in class_names[int(classes[0][i])] and np.array(scores[0][i]) > 0.5:
            print(
                f'{class_names[int(classes[0][i])]}, {np.array(scores[0][i])}, {np.array(boxes[0][i])}')
        else:
            print('No stop sign!')

    img = cv2.cvtColor(img_raw.numpy(), cv2.COLOR_RGB2BGR)
    img = draw_outputs(img, (boxes, scores, classes, nums), class_names)
    cv2.imshow('Detections', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except SystemExit:
        pass
