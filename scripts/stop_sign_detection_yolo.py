#!/usr/bin/env python3
import os

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray
import tensorflow as tf

from yolov3.yolov3_tf2.models import YoloV3Tiny
from yolov3.yolov3_tf2.dataset import transform_images


# Get absolute base path for the folder containing the scripts
base_path = os.path.dirname(os.path.abspath(__file__))


class StopSignDetection:
    def __init__(self, yolo, class_names):
        self.bridge_object = CvBridge()

        # Init the work mode (simulation or real-world)
        self.work_mode = rospy.get_param('~work_mode')

        if self.work_mode == 'simulation':
            # Subscriber which will get images from the topic 'camera/rgb/image_raw'
            self.image_sub = rospy.Subscriber(
                "/camera/rgb/image_raw", Image, self.camera_callback)
        else:
            # Subscriber which will get images from the topic '/raspicam_node/image/compressed'
            self.image_sub = rospy.Subscriber(
                "/raspicam_node/image/compressed", CompressedImage, self.camera_callback)

        # Publisher which will publish to the topic '/stop_sign'
        self.stop_sign_pub = rospy.Publisher('/stop_sign',
                                             Float32MultiArray, queue_size=10)

        # Init the stop sign message
        self.stop_sign_msg = Float32MultiArray()

        # Init the publish rate
        self.rate = rospy.Rate(10)

        # Init the YoloV3 model
        self.yolo = yolo
        self.class_names = class_names

    def camera_callback(self, image):
        if self.work_mode == 'simulation':
            # Select bgr8 because its the OpenCV encoding by default
            img_raw = self.bridge_object.imgmsg_to_cv2(
                image, desired_encoding="bgr8")
        else:
            cv_np_arr = np.fromstring(image.data, np.uint8)
            img_raw = cv2.imdecode(cv_np_arr, cv2.IMREAD_COLOR)

        # Preprocess the image
        img = tf.expand_dims(img_raw, 0)
        img = transform_images(img, 416)

        # Detect objects in the image
        boxes, scores, classes, nums = self.yolo(img)

        # Convert the img to BGR
        img_raw = cv2.cvtColor(img_raw, cv2.COLOR_RGB2BGR)

        stop_signs = []
        # Loop through detections
        for i in range(nums[0]):
            # Get the detected stop sign
            if 'stop sign' in self.class_names[int(classes[0][i])]:
                # Get the probability of the detected stop sign
                probability = scores[0][i].numpy()
                # Convert the representation of the bounding box from proportion to pixel
                wh = np.flip(img_raw.shape[0:2])
                x1 = boxes[0][i][0].numpy() * wh[0]
                y1 = boxes[0][i][1].numpy() * wh[1]
                x2 = boxes[0][i][2].numpy() * wh[0]
                y2 = boxes[0][i][3].numpy() * wh[1]
                stop_signs.append([probability, x1, y1, x2, y2])
            else:
                continue

        if stop_signs:
            # Select the largest stop sign which should be the closest one
            index_largest_stop_sign = 0
            area_largest_stop_sign = 0
            for i, stop_sign in enumerate(stop_signs):
                area = abs((stop_sign[0] - stop_sign[2])
                           * (stop_sign[1] - stop_sign[3]))
                if area >= area_largest_stop_sign:
                    index_largest_stop_sign = i
                    area_largest_stop_sign = area

            # Selected stop sign
            stop_sign = stop_signs[index_largest_stop_sign]

            # Update the msg
            self.stop_sign_msg.data = stop_sign + [area_largest_stop_sign]

        else:
            # Update the msg
            self.stop_sign_msg.data = []

        # Publish
        self.stop_sign_pub.publish(self.stop_sign_msg)
        self.rate.sleep()


def main():
    rospy.init_node('stop_sign_detection', anonymous=True)

    # init YoloV3 model
    yolo = YoloV3Tiny()

    # Load trained weights for tiny-yolo
    yolo.load_weights(os.path.join(
        base_path, 'yolov3/checkpoints/yolov3-tiny.tf')).expect_partial()

    # Load class names that can be detected
    class_names = [c.strip() for c in open(
        os.path.join(base_path, 'yolov3/data/coco.names')).readlines()]

    detection_object = StopSignDetection(yolo, class_names)
    rate = rospy.Rate(10)
    ctrl_c = False

    def shutdownhook():
        # Works better than rospy.is_shutdown()
        rospy.loginfo("Shutdown time!")
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)
    while not ctrl_c:
        rate.sleep()


if __name__ == '__main__':
    main()
