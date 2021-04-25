#!/usr/bin/env python3

# Import Libraries
import cv2
import numpy as np
import rospy

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray
from move_TurtleBot3 import MoveTurtleBot3


class NodeController:
    """
    Node Controller class to coordinate tasks and activities
    """

    def __init__(self, node_name):
        """
        Init function for Node Controller class
        """

        # Creates a node with the specified name and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node(node_name, anonymous=True)

        # Init bridge object
        self.bridge_object = CvBridge()

        # Init the work mode (simulation or real-world)
        self.work_mode = rospy.get_param('~work_mode')

        if self.work_mode == 'simulation':
            # Subscriber which will get images from the topic '/camera/rgb/image_raw'
            self.image_sub = rospy.Subscriber(
                "/camera/rgb/image_raw",
                Image,
                self.camera_callback
            )
        else:
            # Subscriber which will get images from the topic '/raspicam_node/image/compressed'
            self.image_sub = rospy.Subscriber(
                "/raspicam_node/image/compressed",
                CompressedImage,
                self.camera_callback
            )

        # Subscriber which will get obstacle avoidance decision from the topic '/obstacle_avoidance'
        self.obstacle_avoidance_sub = rospy.Subscriber(
            "/obstacle_avoidance",
            Float32MultiArray,
            self.get_obstacle_avoidance_info
        )

        # Subscriber which will get angular velocity from the topic '/line_following' for line following
        self.line_following_sub = rospy.Subscriber(
            "/line_following",
            Float32MultiArray,
            self.get_line_info
        )

        # Subscriber which will get information of stop sign from the topic '/stop_sign'
        self.stop_sign_sub = rospy.Subscriber(
            "/stop_sign",
            Float32MultiArray,
            self.get_stop_sign_info
        )

        # Subscriber which will get information of apriltag from the topic '/apriltag_following'
        self.apriltag_sub = rospy.Subscriber(
            "/apriltag_following",
            Float32MultiArray,
            self.get_apriltag_info
        )

        # Init the velocity message
        self.vel_msg = Twist()

        # Init default linear speed
        self.linear_x = rospy.get_param('~linear_x')

        # Init threshold of stop sign area
        self.area_threshold = rospy.get_param('~area_threshold')

        # Init threshold of stop delay for stop sign
        self.stop_delay = rospy.get_param('~stop_delay')

        # Init method to move the TurtleBot
        self.moveTurtlebot3_object = MoveTurtleBot3()

        # Init obstacle avoidance information
        self.obstacle_avoidance_info = [self.linear_x, 0, 0]

        # Init line following information
        self.line_info = []

        # Init stop sign information
        self.stop_sign_info = []

        # Init apriltag information
        self.apriltag_info = []

        # Init the timer
        self.timer1 = 0
        self.timer2 = 0
        self.mode_timer = 0

        # Init available modes
        self.modes = {
            1: 'obstacle avoidance',
            2: 'line following',
            3: 'tag following'
        }

        # Init default mode to obstacle avoidance
        # and wall following
        self.mode = 1

        # Init stop sign flag
        self.is_stop_sign = False

        # Init the threshold of transition
        self.transition_threshold = 1

        # Init timer for obstacle avoidance
        self.timer_obs_avd = 0


    def mode_decider(self):
        """
        Function to decide the mode
        """
        if self.line_info: 
            # If information is being published to line following
            # topic then set the mode to line following
            self.mode = 2
            self.transition_threshold = 0.1
            return
        if self.apriltag_info and not self.line_info:
            # If information is being published to april tag following
            # topic then set the mode to april tag following
            self.mode = 3
            self.transition_threshold = 20
            return

        # If no mode is being published, default mode to obstacle
        # avoidance and wall following
        self.mode = 1
        self.transition_threshold = 0.1


    def get_line_info(self, msg):
        """
        Function to get data published to the line following topic
        """
        if msg.data:
            self.line_info = msg.data
        else:
            self.line_info = []


    def get_stop_sign_info(self, msg):
        """
        Function to get datapublished to the stop sign detection
        topic
        """
        if msg.data:
            self.stop_sign_info = msg.data
        else:
            self.stop_sign_info = []


    def get_apriltag_info(self, msg):
        """
        Function to get data published to the april tag following
        topic
        """
        if msg.data:
            self.apriltag_info = msg.data
        else:
            self.apriltag_info = []


    def get_obstacle_avoidance_info(self, msg):
        """
        Function to get data published to the obstacle avoidance
        and wall following topic
        """
        self.obstacle_avoidance_info = msg.data


    def draw_detections(self, img):
        """
        Function to draw detections on the feed from the camera.
        Detections include the center of the line detected, the
        stop sign and the april tag
        """
        # Draw detected results
        if self.line_info:
            # Draw the center of detected line
            x_center = int(self.line_info[0])
            y_center = int(self.line_info[1])
            cv2.circle(img, (x_center, y_center), 10, (0, 0, 255), -1)

        if self.stop_sign_info:
            # Draw the detected stop sign
            x1y1 = (int(self.stop_sign_info[1]), int(
                self.stop_sign_info[2]))
            x2y2 = (int(self.stop_sign_info[3]), int(
                self.stop_sign_info[4]))
            img = cv2.rectangle(img, x1y1, x2y2, (255, 0, 0), 2)

        if self.apriltag_info:
            # Draw the detected april tag
            x1y1 = (int(self.apriltag_info[0]), int(
                self.apriltag_info[1]))
            x2y2 = (int(self.apriltag_info[2]), int(
                self.apriltag_info[3]))
            img = cv2.rectangle(img, x1y1, x2y2, (255, 0, 0), 2)

        return img


    def camera_callback(self, image):
        """
        Function for camera callback that is called everytime data is
        published to the camera topic ('/camera/rgb/image_raw' for
        simulated run and '/raspicam_node/image/compressed' for real-life
        run.
        """
        # Threshold of transition
        if self.mode_timer == 0:
            self.mode_timer = rospy.Time.now().to_sec()
        elif rospy.Time.now().to_sec() - self.mode_timer \
                >= self.transition_threshold:
            # Decide mode
            self.mode_decider()
            self.mode_timer = 0

        if self.work_mode == 'simulation':
            # Select bgr8 because its the OpenCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(
                image,
                desired_encoding="bgr8"
            )
        else:
            cv_np_arr = np.frombuffer(image.data, np.uint8)
            cv_image = cv2.imdecode(cv_np_arr, cv2.IMREAD_COLOR)

        # Print mode information on the camera video
        cv_image = cv2.putText(
            cv_image,
            'Mode: ' + self.modes[self.mode],
            (15, 15),
            cv2.FONT_HERSHEY_COMPLEX_SMALL,
            1,
            (0, 0, 255),
            1
        )

        # Draw detected results on camera image
        cv_image = self.draw_detections(cv_image)

        if self.mode == 1:
            # Obstacle Avoidance and Wall Following mode
            self.vel_msg.linear.x = self.obstacle_avoidance_info[0]
            self.vel_msg.angular.z = self.obstacle_avoidance_info[1]
            if self.obstacle_avoidance_info[-2] != 0 \
                    and self.timer_obs_avd == 0:
                self.timer_obs_avd = rospy.Time.now().to_sec()
               
            elif rospy.Time.now().to_sec() - self.timer_obs_avd \
                    > self.obstacle_avoidance_info[-2]:
                self.timer_obs_avd = 0

        elif self.mode == 2:
            # Line Following Mode
            if self.obstacle_avoidance_info[-3] < 0.6:
                self.vel_msg.linear.x = min(
                    self.obstacle_avoidance_info[0],
                    0.05
                )
                self.vel_msg.angular.z = self.obstacle_avoidance_info[1]
            else:
                self.vel_msg.linear.x = self.linear_x/2
                self.vel_msg.angular.z = 0

            # If line is detected
            if self.line_info:
                # Get the angular velocity published by line-follower node
                self.vel_msg.angular.z = self.line_info[-1]

            # If the stop sign is detected
            if self.stop_sign_info:
                # If the stop sign is close (the area is large enough)
                # then change the threshold to 7000 if using
                # stop_sign_detection_yolo
                if self.stop_sign_info[-1] >= self.area_threshold:
                    self.is_stop_sign = True

            if self.is_stop_sign:
                # Keep moving for 18 seconds to ensure the TurtleBot
                # is very close to the stop sign
                if self.timer1 == 0:
                    self.timer1 = rospy.Time.now().to_sec()
                # Change the threshold to 14 if using
                # stop_sign_detection_yolo
                elif rospy.Time.now().to_sec() - self.timer1 \
                        >= self.stop_delay:
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
            # April Tag Following Mode
            if self.apriltag_info:
                # Get the linear and angular velocity published by
                # apriltag-follower node
                self.vel_msg.linear.x = self.apriltag_info[4]
                self.vel_msg.angular.z = self.apriltag_info[5]
                self.last_tag_dir = self.apriltag_info[6] \
                        / abs(self.apriltag_info[6])
            else:
                # Turn in direction where tag was last seen
                # before going out of frame
                self.vel_msg.linear.x = 0

                if self.last_tag_dir > 0:
                    # Tag went out of frame to the right
                    self.vel_msg.angular.z = -0.05
                else:
                    # Tag went out of frame to the left
                    self.vel_msg.angular.z = 0.05

        # Print velocity information on the camera video
        cv_image = cv2.putText(
            cv_image,
            'Vel (x, z): ' + str(round(self.vel_msg.linear.x, 2)) + ',',
            (370, 15),
            cv2.FONT_HERSHEY_COMPLEX_SMALL,
            1,
            (0, 0, 255),
            1
        )

        cv_image = cv2.putText(
            cv_image,
            str(round(self.vel_msg.angular.z, 2)),
            (570, 15),
            cv2.FONT_HERSHEY_COMPLEX_SMALL,
            1,
            (0, 0, 255),
            1
        )

        # Move the TurtleBot
        self.moveTurtlebot3_object.move_robot(self.vel_msg)

        # Show the captured image
        cv2.imshow("Camera", cv_image)
        cv2.waitKey(1)


    def clean_up(self):
        """
        Function to clean up class
        """
        self.moveTurtlebot3_object.clean_class()
        cv2.destroyAllWindows()


def main():
    try:
        node_controller_object = NodeController(
            node_name='main_node'
        )
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

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
