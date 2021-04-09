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
            '/tb3/scan', LaserScan, self.laser_callback)

        self.vel_msg = Float32MultiArray()

        # Default velocity
        self.linear_x = rospy.get_param('~linear_x')
        self.angular_z = -1

        # Init the publish rate
        self.rate = rospy.Rate(10)

        # Init distance reading
        self.current_distance = [100] * 360

    def laser_callback(self, laser_data):
        # Update only if the incoming range data is within the limits
        for i, dist in enumerate(laser_data.ranges):
            if laser_data.range_min < dist < laser_data.range_max:
                self.current_distance[i] = dist

        self.publish_velocity()

    def mean(self, sector):
        """ Take average """
        sum = 0
        for dist in sector:
            sum += dist

        return sum/len(sector)

    def direction_control(self):
        # Segmenting the laser data
        # straight ahead
        fwd = self.current_distance[0]
        # forward view:
        ahead = self.current_distance[-30:-1] + self.current_distance[:30]
        # Left view:
        left = self.current_distance[-60:-30]
        # Right view:
        right = self.current_distance[30:60]

        # Take average
        ahead_mean = self.mean(ahead)
        left_mean = self.mean(left)
        right_mean = self.mean(right)

        # Setting up the Proportional gain values:
        K_left = left_mean / (ahead_mean + right_mean)
        K_right = right_mean / (ahead_mean + left_mean)
        K_ahead = fwd

        # Checking the distance from obstacles and
        # controlling speeds accordingly
        angular_z = K_left * self.angular_z - K_right * self.angular_z
        if ahead_mean > .05:
            linear_x = K_ahead * self.linear_x
        else:
            linear_x = 0

        return [linear_x, angular_z]

    def publish_velocity(self):
        # Keep moving till the robot sees an obstacle at
        # less than safe distance
        self.vel_msg.data = self.direction_control()

        # Publishing our vel_msg
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
