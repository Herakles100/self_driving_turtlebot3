#!/usr/bin/env python3

"""
Script to perform wall following and move Turtlebot around obstacles
"""

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from math import pow, atan2, sqrt


class TurtleBot:

    def __init__(self, node_name):
        # Creates a node with the specified name and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node(node_name, anonymous=True)

        # Publisher which will publish velocity to '/velocity' topic.
        self.vel_pub = rospy.Publisher(
            '/velocity', Float32MultiArray, queue_size=10)

        # Subscriber to subscribe to '/scan' topic
        self.laser_sub = rospy.Subscriber(
            'scan', LaserScan, self.laser_callback)

        self.vel_msg = Float32MultiArray()

        # Default velocity
        self.default_linear_x = rospy.get_param('~linear_x')
        self.default_angular_z = 0

        # Init the publish rate
        self.rate = rospy.Rate(10)

        # Init the minimum distance between turtlebot and obstacle
        self.threshold = 0.5

    def laser_callback(self, msg):
        # Minimum distance of each direction
        front = min(min(msg.ranges[-15:-1] + msg.ranges[:15]), 10)
        fleft = min(min(msg.ranges[-45:-15]), 10)
        fright = min(min(msg.ranges[15:45]), 10)

        if front >= self.threshold:
            linear_x = 0.1
            # Setting up the Proportional gain values:
            K_left = fleft / (front + fright)
            K_right = fright / (front + fleft)
            angular_z = (K_right - K_left) * 1.5
        else:
            linear_x = 0.05
            if fleft < self.threshold and fright < self.threshold:
                angular_z = 0
            else:
                # Setting up the Proportional gain values:
                K_left = fleft / (front + fright)
                K_right = fright / (front + fleft)
                angular_z = (K_right - K_left) * 1.1

        # Publishing vel_msg to topic "/velocity"
        self.vel_msg.data = [linear_x, angular_z]
        self.vel_pub.publish(self.vel_msg)

        # Publish at the desired rate.
        self.rate.sleep()


if __name__ == '__main__':
    try:
        x = TurtleBot(node_name='obstacle_avoidance_node')
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
