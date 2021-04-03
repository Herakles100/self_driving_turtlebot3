#!/usr/bin/env python3
import os

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import tensorflow as tf

from yolov3.yolov3_tf2.models import YoloV3Tiny
from yolov3.yolov3_tf2.dataset import transform_images
from yolov3.yolov3_tf2.utils import draw_outputs


# Get absolute base path for the folder containing the scripts
base_path = os.path.dirname(os.path.abspath(__file__))


class StopSignDetection:
    def __init__(self, yolo, class_names):
        self.bridge_object = CvBridge()

        # Subscriber which will get images from the topic 'camera/rgb/image_raw'
        self.image_sub = rospy.Subscriber(
            "/camera/rgb/image_raw", Image, self.camera_callback)

        # Publisher which will publish to the topic 'camera/rgb/image_raw'
        self.image_pub = rospy.Publisher('/camera/rgb/image_raw',
                                         Image, queue_size=10)

        # Publisher which will publish to the topic '/cmd_vel'.
        self.detection_pub = rospy.Publisher('/stop_sign',
                                             Float32MultiArray, queue_size=10)

        # Init the detection result
        self.detection = Float32MultiArray()

        # Init the publish rate
        self.rate = rospy.Rate(20)

        self.yolo = yolo
        self.class_names = class_names

    def camera_callback(self, image):
        img_raw = self.bridge_object.imgmsg_to_cv2(
            image, desired_encoding="rgb8")

        # Preprocess the image
        img = tf.expand_dims(img_raw, 0)
        img = transform_images(img, 416)

        # Detect objects in the image
        boxes, scores, classes, nums = self.yolo(img)

        # Loop through the detections
        for i in range(nums[0]):
            # Publish when the stop sign is detected
            if 'stop sign' in self.class_names[int(classes[0][i])]:
                self.detection.data = [
                    scores[0][i].numpy()] + boxes[0][i].numpy().tolist()
                img = cv2.cvtColor(img_raw, cv2.COLOR_RGB2BGR)
                img = draw_outputs(
                    img, (boxes, scores, classes, nums), self.class_names)

                # Convert the image to imgmsg
                img_msg = CvBridge().cv2_to_imgmsg(img, 'bgr8')

                # Publish
                self.detection_pub.publish(self.detection)
                self.image_pub.publish(img_msg)
                self.rate.sleep()

                cv2.imshow("Detection", img)
                cv2.waitKey(1)

                break
            else:
                continue

    def clean_up(self):
        cv2.destroyAllWindows()


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
    rate = rospy.Rate(5)
    ctrl_c = False

    def shutdownhook():
        # Works better than rospy.is_shutdown()
        detection_object.clean_up()
        rospy.loginfo("Shutdown time!")
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)
    while not ctrl_c:
        rate.sleep()


if __name__ == '__main__':
    main()
