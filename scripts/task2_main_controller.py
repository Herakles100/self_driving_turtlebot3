#!/usr/bin/env python3
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

from move_TurtleBot3 import MoveTurtleBot3


class NodeController:
    def __init__(self):
        self.bridge_object = CvBridge()

        # Subscriber which will get angular velocity from the topic '/line_following' for line following
        self.line_following_sub = rospy.Subscriber(
            "/line_following", Float32MultiArray, self.get_line_info)

        # Subscriber which will get information of stop sign from the topic '/stop_sign'
        self.stop_sign_sub = rospy.Subscriber(
            "/stop_sign", Float32MultiArray, self.get_stop_sign_info)

        # Subscriber which will get images from the topic '/detected_img'
        self.image_sub = rospy.Subscriber(
            "/detected_img", Image, self.camera_callback)

        # Publisher which will publish to the the topic '/cmd_vel'
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Init the velocity message
        self.vel_msg = Twist()

        # Init the default linear speed
        self.linear_x = rospy.get_param('~linear_x')

        # Init the method to move the TurtleBot
        self.moveTurtlebot3_object = MoveTurtleBot3()

        # Init the line information
        self.line_info = []

        # Init the stop sign information
        self.stop_sign_info = []

        # Init the timer
        self.timer1 = 0
        self.timer2 = 0

        # Init a flag
        self.is_stop_sign = False

    def get_line_info(self, msg):
        if msg.data:
            self.line_info = msg.data
        else:
            self.line_info = []

    def get_stop_sign_info(self, msg):
        if msg.data:
            self.stop_sign_info = msg.data
        else:
            self.stop_sign_info = []

    def camera_callback(self, msg):
        # Select bgr8 because its the OpneCV encoding by default
        cv_image = self.bridge_object.imgmsg_to_cv2(
            msg, desired_encoding="bgr8")

        # Init the default velocity
        self.vel_msg.linear.x = self.linear_x
        self.vel_msg.angular.z = 0

        # If the line is detected, publish the angular velocity determined by the line-following algorithm
        if self.line_info:
            # Draw the center of detected line
            x_center = int(self.line_info[0])
            y_center = int(self.line_info[1])
            cv2.circle(cv_image, (x_center, y_center), 10, (0, 0, 255), -1)

            self.vel_msg.angular.z = self.line_info[-1]

        # If the stop sign is detected
        if self.stop_sign_info:
            # If the stop sign is close to the TurtleBot (the area is large enough)
            if self.stop_sign_info[-1] >= 7000:
                self.is_stop_sign = True

        if self.is_stop_sign:
            # Keep moving for 15 seconds to ensure the TurtleBot is very close to the stop sign
            if self.timer1 == 0:
                self.timer1 = rospy.Time.now().to_sec()
            elif rospy.Time.now().to_sec() - self.timer1 >= 15:
                # Stop the TurtleBot for 3 seconds
                if self.timer2 == 0:
                    self.timer2 = rospy.Time.now().to_sec()
                    self.vel_msg.angular.z = 0
                    self.vel_msg.linear.x = 0
                elif rospy.Time.now().to_sec() - self.timer2 < 3:
                    self.vel_msg.angular.z = 0
                    self.vel_msg.linear.x = 0
                else:
                    self.timer1 = 0
                    self.timer2 = 0
                    # Start to move
                    self.is_stop_sign = False

        # Move the TurtleBot
        self.moveTurtlebot3_object.move_robot(self.vel_msg)

        # Show the captured image
        cv2.imshow("Camera", cv_image)
        cv2.waitKey(1)

    def clean_up(self):
        self.moveTurtlebot3_object.clean_class()
        cv2.destroyAllWindows()


def main():
    rospy.init_node('main_node', anonymous=True)
    node_controller_object = NodeController()
    rate = rospy.Rate(10)
    ctrl_c = False

    def shutdownhook():
        # Works better than rospy.is_shutdown()
        node_controller_object.clean_up()
        rospy.loginfo("Shutdown time!")
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)
    while not ctrl_c:
        rate.sleep()


if __name__ == '__main__':
    main()
