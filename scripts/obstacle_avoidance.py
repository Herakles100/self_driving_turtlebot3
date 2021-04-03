#!/usr/bin/env python3

"""
Script to perform wall following and move Turtlebot around obstacles
"""

import rospy
import cv2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from math import pow, atan2, sqrt


class TurtleBot:

    def __init__(self, node_name):
        # Creates a node with the specified name and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node(node_name, anonymous=True)

        # Publisher which will publish velocity to '/cmd_vel' topic.
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # Subscriber to subscribe to '/scan' topic
        self.laser_sub = rospy.Subscriber('scan', LaserScan, self.laser_callback)

        self.bridge_object = CvBridge()

        self.image_sub = rospy.Subscriber(
            #"/raspicam_node/image/compressed", Image, self.camera_callback)
            "/camera/rgb/image_raw", Image, self.camera_callback)

        self.rate = rospy.Rate(10)

        # initialise a distance reading
        self.current_distance=[100]*360

    def camera_callback(self, camera_data):
        cv_image = self.bridge_object.imgmsg_to_cv2(
            camera_data,
            desired_encoding="bgr8"
        )
        height, width,_= cv_image.shape
        cv2.imshow("Camera Feed", cv_image)
        cv2.waitKey(1)

    def laser_callback(self, laser_data):

        # update only if the incoming range data is within the limits
        for i in range(0,360):
            if laser_data.ranges[i]>laser_data.range_min and laser_data.ranges[i]<laser_data.range_max:
                self.current_distance[i] = laser_data.ranges[i]
            else:
                self.current_distance[i] = self.current_distance[i]

    def mean(self, sector):
        sum=0
        for i in range(len(sector)):
            sum=sum+sector[i]
        mean=sum/len(sector)
        return mean

    def direction_control(self):

        # Segmenting the laser data
        # straight ahead
        fwd=self.current_distance[0]

        # forward view:
        ahead=[]
        for i in range(-30,30):
            ahead.append(self.current_distance[i])

        # Left view:
        left=self.current_distance[-60:-30]

        # Right view:
        right=self.current_distance[30:60]

        ahead_mean=self.mean(ahead)
        left_mean=self.mean(left)
        right_mean=self.mean(right)

        # Setting up the Proportional gain values:
        K_left=left_mean/(ahead_mean+right_mean)
        K_right=right_mean/(ahead_mean+left_mean)
        K_ahead=fwd
        # Defining the base speed of the turtlebot3
        omega0 = -1
        speed0 = .1

        # Checking the distance from obstacles and controlling speeds accordingly
        if ahead_mean > .05:
            omega = K_left*omega0 - K_right*omega0
            speed = K_ahead*speed0
        else:
            omega = K_left*omega0 - K_right*omega0
            speed = 0

        return [speed,omega]

    def move(self):
        """Moves the turtle while avoiding obstacles"""
        
        vel_msg = Twist()
                                
        # Keep moving till the robot sees an obstacle at less than safe distance
        while not rospy.is_shutdown():
            vel_msg.linear.x = self.direction_control()[0]
            vel_msg.angular.z = self.direction_control()[1]

            # Publishing our vel_msg
            self.vel_pub.publish(vel_msg)
             # Publish at the desired rate.
            self.rate.sleep()
            
        # Stopping our robot
        vel_msg.linear.x = 0
        self.vel_pub.publish(vel_msg)

        # If we press control + C, the node will stop.
        rospy.spin()

if __name__ == '__main__':
    try:
        x = TurtleBot(node_name='turtlebot3_controller')
        x.move()
    except rospy.ROSInterruptException: pass
