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
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
#from move_TurtleBot3 import MoveTurtleBot3
from PID_controller import PID
from apriltag_ros.msg import AprilTagDetectionArray

class TagFollower:
    def __init__(self):
        self.bridge_object = CvBridge()
        
        self.image_sub = rospy.Subscriber(
            "tb3/camera/rgb/image_raw/compressed", Image, self.camera_callback)
        self.tagdetect_sub = rospy.Subscriber(
            "/tag_detections", AprilTagDetectionArray, self.detect_callback)
        
        self.tag_follow_pub = rospy.Publisher("/tag_follow_instruction",Float32MultiArray, queue_size=10)
        # Init the line-following message
        self.tag_follow_msg = Float32MultiArray()

        self.listener = tf.TransformListener()
        self.twist_object = Twist()

        # Init detected_tag value
        self.detected_tag = False

        # Init PID controller
        Kp = rospy.get_param('~Kp')
        Ki = rospy.get_param('~Ki')
        Kd = rospy.get_param('~Kd')
        self.pid_object = PID(Kp, Ki, Kd)

    def tf_follow(self):
        detector=apriltag.Detector()

    def detect_callback(self,msg):
        if len(msg.detections) > 0:
            self.detected_tag = True
        else:
            self.detected_tag = False
    
    def camera_callback(self,data):
        if self.detected_tag:
            # If a tag is detected
            # get the X position of the tag_id of concern wrt camera frame
            trans,rot=self.listener.lookupTransform('tag_0', 'camera', rospy.Time(0))
            cx=trans[0]
            cz=trans[2]

            # PID Controller
            # Move  only if tag is more than 0.3 units ahead
            if cz>0.3:
                self.twist_object.linear.x = 0.05
                # Controlling the angular velocity
                error_x = -cx*100
                self.twist_object.angular.z = self.pid_object.update(error_x) / 200
            else:
                #If a tag is detected at less than 0.3 units
                self.twist_object.linear.x = 0
                self.twist_object.angular.z = 0

            # Publish the velocity data to /tag_follow_instruction
            self.tag_follow_msg.data = [self.twist_object.linear.x, self.twist_object.angular.z]
            self.tag_follow_pub.publish(self.tag_follow_msg)

        else:
            # If no tag found, rotate in place
            self.twist_object.linear.x = 0
            self.twist_object.angular.z = 0.05           
            # Publish the velocity data to /tag_follow_instruction
            self.tag_follow_msg.data = [self.twist_object.linear.x, self.twist_object.angular.z]
            self.tag_follow_pub.publish(self.tag_follow_msg)


    def clean_up(self):
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
