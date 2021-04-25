#!/usr/bin/env python3

# Import libraries
import rospy
from geometry_msgs.msg import Twist


class MoveTurtleBot3:
    """
    Move TurtleBot3 class for moving turtlebot3
    """

    def __init__(self):
        """
        Init function for MoveTurtleBot3 class
        """

        # Publisher to publish velocity data to the topic '/cmd_vel'
        self.cmd_vel_pub = rospy.Publisher(
            '/cmd_vel', Twist, queue_size=10
        )

        # Subscriber to get velocity data from the topic '/cmd_vel'
        self.cmd_vel_subs = rospy.Subscriber(
            '/cmd_vel', Twist, self.cmdvel_callback
        )

        # Init Twist message for last cmd_vel
        self.last_cmdvel_command = Twist()

        # Init the publish rate
        self.cmd_vel_pub_rate = rospy.Rate(10)


    def cmdvel_callback(self, msg):
        """
        Callback function called when data is published to the
        '/cmd_vel' topic
        """
        self.last_cmdvel_command = msg


    def compare_twist_commands(self, twist1, twist2):
        """
        Function to compare current Twist message with previous
        Twist message
        """
        LX = twist1.linear.x == twist2.linear.x
        LY = twist1.linear.y == twist2.linear.y
        LZ = twist1.linear.z == twist2.linear.z
        AX = twist1.angular.x == twist2.angular.x
        AY = twist1.angular.y == twist2.angular.y
        AZ = twist1.angular.z == twist2.angular.z
        equal = LX and LY and LZ and AX and AY and AZ

        if not equal:
            rospy.logwarn(
                "The Current Twist is not the same as the one sent, "
                "Resending"
            )
        return equal


    def move_robot(self, twist_object):
        """
        Function to move the robot by publishing data to the
        '/cmd_vel' topic
        """
        # We make this to avoid Topic loss, specially at the start
        current_equal_to_new = False
        while not current_equal_to_new:
            self.cmd_vel_pub.publish(twist_object)
            self.cmd_vel_pub_rate.sleep()
            current_equal_to_new = \
                self.compare_twist_commands(
                    twist1=self.last_cmdvel_command,
                    twist2=twist_object
                )


    def clean_class(self):
        """
        Function to clean class components
        """
        # Stop the Robot
        twist_object = Twist()
        twist_object.angular.z = 0.0
        self.move_robot(twist_object)
