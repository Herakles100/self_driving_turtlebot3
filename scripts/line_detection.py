#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

from PID_controller import PID


class LineFollower:
    def __init__(self):
        self.bridge_object = CvBridge()

        # Subscriber which will get images from the topic 'camera/rgb/image_raw'
        self.image_sub = rospy.Subscriber(
            "/tb3/camera/rgb/image_raw", Image, self.camera_callback)

        # Publisher which will publish to the the topic '/line_following'
        self.line_following_pub = rospy.Publisher(
            "/line_following", Float32MultiArray, queue_size=10)

        # Init the publish rate
        self.rate = rospy.Rate(10)

        # Init the line-following message
        self.line_following_msg = Float32MultiArray()

        # Init PID controller
        Kp = rospy.get_param('~Kp')
        Ki = rospy.get_param('~Ki')
        Kd = rospy.get_param('~Kd')
        self.pid_object = PID(Kp, Ki, Kd)

        # Center shift
        self.center_shift = rospy.get_param('~line_center_shift')

        # Init the lower bound and upper bound of the specific color
        self.lower_HSV = np.array(eval(rospy.get_param('~lower_HSV')))
        self.upper_HSV = np.array(eval(rospy.get_param('~upper_HSV')))

    def camera_callback(self, data):
        # Select bgr8 because its the OpenCV encoding by default
        cv_image = self.bridge_object.imgmsg_to_cv2(
            data, desired_encoding="bgr8")

        # Crop the parts of the image we don't need
        height, width, _ = cv_image.shape
        upper_bound, lower_bound = 180, 230
        crop_img = cv_image[int(height/2) +
                            upper_bound:int(height/2) + lower_bound][:]

        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Threshold the HSV image to get only specific colors
        mask = cv2.inRange(hsv, self.lower_HSV, self.upper_HSV)

        # Find the centroid
        m = cv2.moments(mask, False)
        try:
            cx, cy = int(m['m10']/m['m00']), int(m['m01']/m['m00'])
            found_blob = True
        except ZeroDivisionError:
            found_blob = False

        # PID Controller
        if found_blob:
            # Determine the angular velocity
            error = cx - width / 2 + self.center_shift
            angular_vel = self.pid_object.update(error) / 450

            # Update the msg
            self.line_following_msg.data = [
                cx, cy + height / 2 + upper_bound, angular_vel]
            # Publish
            self.line_following_pub.publish(self.line_following_msg)
            self.rate.sleep()
        else:
            # Update the msg
            self.line_following_msg.data = []
            # Publish
            self.line_following_pub.publish(self.line_following_msg)
            self.rate.sleep()


def main():
    rospy.init_node('line_following_node', anonymous=True)
    line_follower_object = LineFollower()
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
