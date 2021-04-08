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
        # self.default_linear_x = rospy.get_param('~linear_x')
        self.linear_x = 0.1

        # Init the publish rate
        self.rate = rospy.Rate(10)

        # Init minimum distance in each direction
        self.front = 0
        self.fleft = 0
        self.fright = 0

        # Init the minimum distance between turtlebot and obstacle
        self.front_threshold = 0.2
        self.left_threshold = 0.1
        self.right_threshold = 0.15

        # Init action space
        self.actions = {
            'dynamic_steering': [self.linear_x, 0],
            'move_straight': [self.linear_x, 0],
            'turn_left': [0, -0.1],
            'turn_right': [0, 0.1],
            'stop': [0, 0]
        }

    def _cal_angular_z(self):
        K_left = self.fleft / (self.front + self.fright)
        K_right = self.fright / (self.front + self.fleft)
        angular_z = (K_right - K_left) * 1.5

        return angular_z

    def laser_callback(self, msg):
        # Minimum distance in each direction
        self.front = min(min(msg.ranges[-15:-1] + msg.ranges[:15]), 10)
        self.fleft = min(min(msg.ranges[-90:-15]), 10)
        self.fright = min(min(msg.ranges[15:90]), 10)

        if self.front > self.front_threshold:
            if self.fleft > self.left_threshold and self.fright > self.right_threshold:
                self.actions['dynamic_steering'][-1] = self._cal_angular_z()
                action = 'dynamic_steering'
            elif self.fleft > self.left_threshold and self.fright < self.right_threshold:
                action = 'turn_left'
            elif self.fleft < self.left_threshold and self.fright > self.right_threshold:
                action = 'turn_right'
            else:
                action = 'move_straight'
        else:
            if self.fleft > self.left_threshold and self.fright > self.right_threshold:
                action = 'turn_left'
            elif self.fleft > self.left_threshold and self.fright < self.right_threshold:
                action = 'turn_left'
            elif self.fleft < self.left_threshold and self.fright > self.right_threshold:
                action = 'turn_right'
            else:
                action = 'stop'

        # Publishing vel_msg to topic "/velocity"
        self.vel_msg.data = self.actions[action]
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
