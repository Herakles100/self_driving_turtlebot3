#!/usr/bin/env python3

# Import libraries
import cv2
import numpy as np
import rospy

from apriltag import apriltag
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32MultiArray
from pid_controller import PID


class TagFollower:
    """
    April Tag Follower class for april tag tracking
    """

    def __init__(self, node_name):
        """
        Init function for TagFollower class
        """

        # Creates a node with the specified name and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node(node_name, anonymous=True)

        # Init bridge object
        self.bridge_object = CvBridge()

        # Init the work mode (simulation or real-world)
        self.work_mode = rospy.get_param('~work_mode')

        if self.work_mode == 'simulation':
            # Subscriber which will get images from the topic
            # 'camera/rgb/image_raw'
            self.image_sub = rospy.Subscriber(
                "/camera/rgb/image_raw",
                Image,
                self.camera_callback
            )
        else:
            # Subscriber which will get images from the topic
            # '/raspicam_node/image/compressed'
            self.image_sub = rospy.Subscriber(
                "/raspicam_node/image/compressed",
                CompressedImage,
                self.camera_callback
            )

        # Publisher to publish data for april tag following to
        # the topic '/apriltag_following'
        self.tag_follow_pub = rospy.Publisher(
            "/apriltag_following", Float32MultiArray, queue_size=10
        )

        # Init threshold of april tag area
        self.area_threshold = rospy.get_param('~tag_area_threshold')

        # Init the line-following message
        self.tag_follow_msg = Float32MultiArray()

        # Init the publish rate
        self.rate = rospy.Rate(10)

        # Init the default linear speed
        self.default_linear_x = rospy.get_param('~linear_x')

        # Init april tag family
        self.tag_family = rospy.get_param('~tag_family')


    def camera_callback(self, image):
        """
        Function for camera callback that is called everytime data is
        published to the camera topic ('/camera/rgb/image_raw' for
        simulated run and '/raspicam_node/image/compressed' for real
        world run.
        """
        if self.work_mode == 'simulation':
            # Select bgr8 because its the OpenCV encoding by default
            img_raw = self.bridge_object.imgmsg_to_cv2(
                image, desired_encoding="bgr8"
            )
        else:
            cv_np_arr = np.frombuffer(image.data, np.uint8)
            img_raw = cv2.imdecode(cv_np_arr, cv2.IMREAD_COLOR)

        _, width, _ = img_raw.shape

        gray_image = cv2.cvtColor(img_raw, cv2.COLOR_BGR2GRAY)

        # Detect April tag
        detector = apriltag(self.tag_family)
        tags = detector.detect(gray_image)

        if tags:
            # Pick the largest tag
            index_largest_tag = 0
            largest_area = 0
            for i, tag in enumerate(tags):
                # Get the four corners
                _, right_bottom, _, left_top = tag['lb-rb-rt-lt']
                x1, y1 = left_top[0], left_top[1]
                x2, y2 = right_bottom[0], right_bottom[1]
                area = abs((x1 - x2) * (y1 - y2))
                if area > largest_area:
                    index_largest_tag = i
                    largest_area = area

            tag = tags[index_largest_tag]

            # Get the four corners
            _, right_bottom, _, left_top = tag['lb-rb-rt-lt']
            x1, y1 = left_top[0], left_top[1]
            x2, y2 = right_bottom[0], right_bottom[1]

            cx = abs(x1 - x2) / 2 + x1
            # Controlling the angular velocity
            error_x = cx - width / 2
            angular_z = error_x / -2000

            if largest_area < self.area_threshold:
                linear_x = self.default_linear_x
            else:
                linear_x = 0

            # Create the msg to publish
            self.tag_follow_msg.data = [
                x1, y1,
                x2, y2,
                linear_x, angular_z,
                error_x
            ]
        else:
            # Update the msg
            self.tag_follow_msg.data = []

        # Publish the data to '/apriltag_following'
        self.tag_follow_pub.publish(self.tag_follow_msg)
        self.rate.sleep()


def main():
    try:
        tag_detection_object = TagFollower(
            node_name='tag_following_node'
        )
        rate = rospy.Rate(10)
        ctrl_c = False

        def shutdownhook():
            # Works better than rospy.is_shutdown()
            rospy.loginfo("Shutdown time!")
            ctrl_c = True

        rospy.on_shutdown(shutdownhook)
        while not ctrl_c:
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
