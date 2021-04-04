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

        # Subscriber which will get images from the topic 'camera/rgb/image_raw'
        self.image_sub = rospy.Subscriber(
            "/camera/rgb/image_raw", Image, self.camera_callback)

        # Publisher which will publish to the the topic '/cmd_vel'
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Init the velocity message
        self.vel_msg = Twist()

        # Init the method to move the TurtleBot
        self.moveTurtlebot3_object = MoveTurtleBot3()

        # Init the line information
        self.line_info = []

        # Init the stop sign information
        self.stop_sign_info = []

        # Init the timer
        self.timer = 0

    def get_line_msg(self, msg):
        if msg.data:
            self.line_info = msg.data

    def get_stop_sign_info(self, msg):
        if msg.data:
            self.stop_sign_info = msg.data

    def camera_callback(self, msg):
        # Select bgr8 because its the OpneCV encoding by default
        cv_image = self.bridge_object.imgmsg_to_cv2(
            msg, desired_encoding="bgr8")

        # Init the default velocity
        self.vel_msg.linear.x = 0.2
        self.vel_msg.angular.z = 0

        # If the line is detected, publish the angular velocity determined by the line-following algorithm
        if self.line_info:
            # Draw the center of detected line
            height, *_ = cv_image.shape
            x_center = int(self.line_info[0] + height / 2 + 50)
            y_center = int(self.line_info[0])
            cv2.circle(cv_image, (x_center, y_center), 10, (0, 0, 255), -1)

            self.vel_msg.angular.z = self.line_info[-1]

        # If the stop sign is detected
        if self.stop_sign_info:
            # Draw the detected stop sign
            prob = self.stop_sign_info[0]
            x1y1 = (int(self.stop_sign_info[1]), int(self.stop_sign_info[2]))
            x2y2 = (int(self.stop_sign_info[3]), int(self.stop_sign_info[4]))
            cv_image = cv2.rectangle(cv_image, x1y1, x2y2, (255, 0, 0), 2)
            cv_image = cv2.putText(cv_image, 'stop sign {:.4f}'.format(
                prob), x1y1, cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 255), 2)

            # If the stop sign is close to the TurtleBot (the area is large enough)
            if self.stop_sign_info[-1] >= 3500:
                # Stop the TurtleBot and start counting down
                if self.timer == 0 or rospy.Time.now().to_sec() - self.timer < 3:
                    self.timer = rospy.Time.now().to_sec()
                    self.vel_msg.angular.z = 0
                    self.vel_msg.linear.x = 0
                # Start to move after stopping for 3 seconds
                elif rospy.Time.now().to_sec() - self.timer >= 3:
                    pass
                # Start to detect the stop sign after 6 seconds
                elif rospy.Time.now().to_sec() - self.timer >= 6:
                    self.timer = 0

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
    rate = rospy.Rate(5)
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
