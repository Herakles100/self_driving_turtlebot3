#!/usr/bin/env python3
import cv2
import numpy as np
from apriltag import apriltag
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray

from PID_controller import PID


class TagFollower:
    def __init__(self):
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

        self.tag_follow_pub = rospy.Publisher(
            "/apriltag_following", Float32MultiArray, queue_size=10)

        # Init the line-following message
        self.tag_follow_msg = Float32MultiArray()

        # Init the publish rate
        self.rate = rospy.Rate(10)

        # Init the default linear speed
        self.default_linear_x = rospy.get_param('~linear_x')

        # Init april tag family
        self.tag_family = rospy.get_param('~tag_family')

    def camera_callback(self, image):
        if self.work_mode == 'simulation':
            # Select bgr8 because its the OpenCV encoding by default
            img_raw = self.bridge_object.imgmsg_to_cv2(
                image, desired_encoding="bgr8")
        else:
            cv_np_arr = np.fromstring(image.data, np.uint8)
            img_raw = cv2.imdecode(cv_np_arr, cv2.IMREAD_COLOR)

        _, width, _ = img_raw.shape

        gray_image = cv2.cvtColor(img_raw, cv2.COLOR_BGR2GRAY)

        # Detect
        detector = apriltag(self.tag_family)
        tags = detector.detect(gray_image)

        if tags:
            # Pick the largest tag
            index_largest_tag = 0
            largest_are = 0
            for i, tag in enumerate(tags):
                # Get four corners
                _, right_bottom, _, left_top = tag['lb-rb-rt-lt']
                x1, y1 = left_top[0], left_top[1]
                x2, y2 = right_bottom[0], right_bottom[1]
                area = abs((x1 - x2) * (y1 - y2))
                if area > largest_are:
                    index_largest_tag = i
                    largest_are = area

            tag = tags[index_largest_tag]

            # Get four corners
            _, right_bottom, _, left_top = tag['lb-rb-rt-lt']
            x1, y1 = left_top[0], left_top[1]
            x2, y2 = right_bottom[0], right_bottom[1]

            cx = abs(x1 - x2) / 2 + x1
            # Controlling the angular velocity
            error_x = cx - width / 2
            angular_z = error_x / -2000

            if largest_are < 10000 and abs(angular_z) < 0.05:
                linear_x = self.default_linear_x
            else:
                linear_x = 0

            # Create the msg to publish
            self.tag_follow_msg.data = [x1, y1, x2, y2, linear_x, angular_z]
        else:
            # Update the msg
            self.tag_follow_msg.data = []

        # Publish the velocity data to /tag_follow_instruction
        self.tag_follow_pub.publish(self.tag_follow_msg)
        self.rate.sleep()


def main():
    rospy.init_node('tag_following_node', anonymous=True)
    tag_detection_object = TagFollower()
    rate = rospy.Rate(10)
    ctrl_c = False

    def shutdownhook():
        # Works better than rospy.is_shutsaving strings in pythondown()
        rospy.loginfo("Shutdown time!")
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)
    while not ctrl_c:
        rate.sleep()


if __name__ == '__main__':
    main()
