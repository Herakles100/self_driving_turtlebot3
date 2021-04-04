#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

from move_TurtleBot3 import MoveTurtleBot3
from PID_controller import PID


class LineFollower:
    def __init__(self):
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/camera/rgb/image_raw", Image, self.camera_callback)
        self.linefollow_pub = rospy.Publisher("/line_following", Float32MultiArray, queue_size=10)
        self.line_following_msg = Float32MultiArray()
        self.line_following_msg = []
        
        self.moveTurtlebot3_object = MoveTurtleBot3()

        self.twist_object = Twist()

        # Init PID controller
        Kp = rospy.get_param('~Kp')
        Ki = rospy.get_param('~Ki')
        Kd = rospy.get_param('~Kd')
        self.pid_object = PID(Kp, Ki, Kd)

        # Init the lower bound and upper bound of the specific color
        self.lower_HSV = np.array(eval(rospy.get_param('~lower_HSV')))
        self.upper_HSV = np.array(eval(rospy.get_param('~upper_HSV')))

    def camera_callback(self, data):
        # Select bgr8 because its the OpenCV encoding by default
        cv_image = self.bridge_object.imgmsg_to_cv2(
            data, desired_encoding="bgr8")

        # Crop the parts of the image we don't need
        height, width, _ = cv_image.shape
        crop_img = cv_image[int(height/2)+50:int(height/2)+100][:]

        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Threshold the HSV image to get only specific colors
        mask = cv2.inRange(hsv, self.lower_HSV, self.upper_HSV)

        # Find all centroids
        contours, _ = cv2.findContours(
            mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)
        centers = []
        for contour in contours:
            m = cv2.moments(contour)
            try:
                center = (int(m['m10']/m['m00']), int(m['m01']/m['m00']))
                centers.append(center)
                cv2.circle(mask, center, 10, (0, 0, 255), -1)
            except ZeroDivisionError:
                pass

        # Select the most left centroid so that the TurtleBot will track the most left path
        most_left_centroid_index = 0
        index = 0
        min_x_val = 0
        for center in centers:
            cx = center[0]
            if cx <= min_x_val:
                min_x_val = cx
                most_left_centroid_index = index
            index += 1

        try:
            cx = centers[most_left_centroid_index][0]
            cy = centers[most_left_centroid_index][1]
            found_blob = True
        except:
            cx = int(width/2)
            cy = int(height/2)
            found_blob = False

        cv2.imshow("Original", cv_image)
        cv2.imshow("MASK", mask)
        cv2.waitKey(1)

        # PID Controller
        self.twist_object.linear.x = 0.2
        if found_blob:
            error = cx - width / 2
            self.twist_object.angular.z = self.pid_object.update(error) / 200
        else:
            self.twist_object.angular.z = 0

        self.line_following_msg = [cx,cy,self.twist_object.angular.z]

        self.linefollow_pub.publish(line_following_msg)
        # Make it start moving
        #self.moveTurtlebot3_object.move_robot(self.twist_object)

    def clean_up(self):
        self.moveTurtlebot3_object.clean_class()
        cv2.destroyAllWindows()


def main():
    rospy.init_node('line_following_node', anonymous=True)
    line_follower_object = LineFollower()
    rate = rospy.Rate(5)
    ctrl_c = False

    def shutdownhook():
        # Works better than rospy.is_shutdown()
        line_follower_object.clean_up()
        rospy.loginfo("Shutdown time!")
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)
    while not ctrl_c:
        rate.sleep()


if __name__ == '__main__':
    main()
