#!/usr/bin/env python3
import cv2
import apriltag
import apriltag_ros
import numpy as np
import rospy
import math
import tf2_ros
import tf
import geometry_msgs.msg
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage as Image
from geometry_msgs.msg import Twist

from move_TurtleBot3 import MoveTurtleBot3
from PID_controller import PID


class TagFollower:
    def __init__(self):
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/raspicam_node/image/compressed", Image, self.camera_callback)
        self.listener = tf.TransformListener()
        self.moveTurtlebot3_object = MoveTurtleBot3()
        self.twist_object = Twist()

        # Init PID controller
        Kp = rospy.get_param('~Kp')
        Ki = rospy.get_param('~Ki')
        Kd = rospy.get_param('~Kd')
        self.pid_object = PID(Kp, Ki, Kd)

    def tf_follow(self):
        detector=apriltag.Detector()
    
    def camera_callback(self, data):
        # Select bgr8 because its the OpneCV encoding by default
        cv_np_arr = np.fromstring(data.data, np.uint8)
        cv_image = cv2.imdecode(cv_np_arr, cv2.IMREAD_COLOR)
        height, width,_= cv_image.shape
        cv2.imshow("Original", cv_image)
        cv2.waitKey(1)

        # get the X position of the tag_id of concern wrt camera frame
        trans,rot=self.listener.lookupTransform('tag_1', 'camera', rospy.Time(0))
        cx=trans[0]

        # PID Controller
        self.twist_object.linear.x = 0.05
        error = -cx*100     # Negative sign is for sign convention of frame coordinates

        self.twist_object.angular.z = self.pid_object.update(error) / 200
        self.moveTurtlebot3_object.move_robot(self.twist_object)

    def clean_up(self):
        self.moveTurtlebot3_object.clean_class()
        cv2.destroyAllWindows()


def main():
    rospy.init_node('tag_following_node', anonymous=True)
    line_follower_object = TagFollower()
    rate = rospy.Rate(5)
    ctrl_c = False

    def shutdownhook():
        # Works better than rospy.is_shutsaving strings in pythondown()
        line_follower_object.clean_up()
        rospy.loginfo("Shutdown time!")
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)
    while not ctrl_c:
        rate.sleep()


if __name__ == '__main__':
    main()
