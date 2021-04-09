#!/usr/bin/env python3
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
#from std_msgs.msg import Int8
from apriltag_ros.msg import AprilTagDetectionArray

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

        # Subscriber which will get images from the topic '/camera/rgb/image_raw'
        self.image_sub = rospy.Subscriber(
            "/tb3/camera/rgb/image_raw", Image, self.camera_callback)

        # Subscriber which will get  info from /tag_follow_instruction, only if tag is detected
        self.tag_follow_sub = rospy.Subscriber(
            "/tag_follow_instruction", Float32MultiArray, self.get_tag_info)

        self.tag_detect_sub = rospy.Subscriber(
            "/tag_detections", AprilTagDetectionArray, self.get_tag_flag)

        # Subscriber which will get velocity from the topic '/velocity'
        self.vel_sub = rospy.Subscriber(
            "/velocity", Float32MultiArray, self.get_velocity_info)

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

        # Init the velocity information
        self.velocity_info = [self.linear_x, 0]

        # Init the tag information
        self.tag_vel = []

        # Init the timer
        self.timer1 = 0
        self.timer2 = 0
        self.mode_timer = 0

        # Init mode
        self.mode = 1  # Default to obstacle avoidance and wall following

        # Init a flag
        self.is_stop_sign = False

        #Init a flag for tag detection
        self.is_tag_detected = False

        # Init modes
        self.modes = {
            1: 'obstacle avoidance',
            2: 'line following',
            3: 'tag following'
        }

    def mode_decider(self):
        if self.line_info and self.is_tag_detected:
            self.mode = 2  # line following until no line detected
            return
        if self.line_info:  # line following
            self.mode = 2
            return
        if self.is_tag_detected:
            self.mode = 3  # tag following
            return
        self.mode = 1  # if no mode being published, default to obstacle and wall mode

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

    def get_velocity_info(self, msg):
        self.velocity_info = msg.data

    def get_tag_info(self,msg):
        self.tag_vel = msg.data

    def get_tag_flag(self,msg):
        if len(msg.detections) > 0:
            self.is_tag_detected = True
        else:
            self.is_tag_detected = False

    def camera_callback(self, msg):
        if self.mode_timer == 0:
            self.mode_timer = rospy.Time.now().to_sec()
        elif rospy.Time.now().to_sec() - self.mode_timer >= 1:
            # Decide mode
            self.mode_decider()
            self.mode_timer = 0

        # Select bgr8 because its the OpneCV encoding by default
        cv_image = self.bridge_object.imgmsg_to_cv2(
            msg, desired_encoding="bgr8")

        # Print mode information on the camera video
        cv_image = cv2.putText(cv_image, 'Mode: ' + self.modes[self.mode],
                               (15, 15), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1,
                               (0, 0, 255), 1)

        # Init the default velocity
        self.vel_msg.linear.x = self.linear_x
        self.vel_msg.angular.z = 0

        if self.mode == 1:
            self.vel_msg.linear.x = self.velocity_info[0]
            self.vel_msg.angular.z = self.velocity_info[1]

        elif self.mode == 2:
            # This is line following scenario
            # If the line is detected, publish the angular velocity determined by the line-following algorithm
            if self.line_info:
                # Draw the center of detected line
                x_center = int(self.line_info[0])
                y_center = int(self.line_info[1])
                cv2.circle(cv_image, (x_center, y_center), 10, (0, 0, 255), -1)

                # Get the angular velocity publushed by line-follower node
                self.vel_msg.angular.z = self.line_info[-1]

            # If the stop sign is detected
            if self.stop_sign_info:
                # Draw the detected stop sign
                x1y1 = (int(self.stop_sign_info[1]), int(
                    self.stop_sign_info[2]))
                x2y2 = (int(self.stop_sign_info[3]), int(
                    self.stop_sign_info[4]))
                cv_image = cv2.rectangle(cv_image, x1y1, x2y2, (255, 0, 0), 2)

                # If the stop sign is close to the TurtleBot (the area is large enough)
                # Change the threshold to 7000 if using stop_sign_detection_yolo
                if self.stop_sign_info[-1] >= 3300:
                    self.is_stop_sign = True

            if self.is_stop_sign:
                # Keep moving for 18 seconds to ensure the TurtleBot is very close to the stop sign
                if self.timer1 == 0:
                    self.timer1 = rospy.Time.now().to_sec()
                # Change the threshold to 14 if using stop_sign_detection_yolo
                elif rospy.Time.now().to_sec() - self.timer1 >= 18:
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

        elif self.mode == 3:
            """ This is april tag following scenario """
            self.vel_msg.linear.x = self.tag_vel[0]
            self.vel_msg.angular.z = self.tag_vel[1]

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
