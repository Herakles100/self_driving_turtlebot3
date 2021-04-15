#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from tf.transformations import euler_from_quaternion


class TurtleBot:
    def __init__(self, node_name):
        # Creates a node with the specified name and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node(node_name, anonymous=True)

        # Init the work mode (simulation or real-world)
        self.work_mode = rospy.get_param('~work_mode')

        # Publisher which will publish obstacle avoidance decision to '/obstacle_avoidance' topic.
        self.obstacle_avoidance_pub = rospy.Publisher(
            '/obstacle_avoidance', Float32MultiArray, queue_size=10)

        # Init the publish rate
        self.rate = rospy.Rate(10)

        # Subscriber to subscribe from '/scan' topic
        self.lidar_sub = rospy.Subscriber(
            '/scan', LaserScan, self.laser_callback)

        # Subscriber to subscribe from '/odom' topic
        self.odom_sub = rospy.Subscriber(
            '/odom', Odometry, self.odom_callback)

        # Init the msg to publish
        self.obs_avoid_dec_msg = Float32MultiArray()

        # Default velocity
        self.linear_x = rospy.get_param('~linear_x')

        # Init distance threshold
        self.threshold = rospy.get_param('~obstacle_distance_threshold')

        # Init gamma for attractive potential field
        self.gamma = rospy.get_param('~gamma')

        # Init param of proportional control
        self.p = rospy.get_param('~param_proportional_control')

        # Init the turtlebot's width
        self.robot_width = rospy.get_param('~turtlebot_width')

        # Init the maximum range of lidar
        self.d_max = rospy.get_param('~max_lidar_range')

        # Init the goal
        self.theta_goal = rospy.get_param('~theta_goal')

        # Init the yaw angle in degrees
        self.yaw_angle = 0

        # Init theta_i which represents choices
        self.theta_is = list(range(-90, 91))

    def odom_callback(self, odom_data):
        orientation_q = odom_data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y,
                            orientation_q.z, orientation_q.w]
        *_, self.yaw_angle = euler_from_quaternion(orientation_list)

    def laser_callback(self, laser_data):
        # Pick out the data in the specific view range
        distances = laser_data.ranges[-90:-1] + laser_data.ranges[:90]

        # Detect all obstacles within the threshold
        d_k, sigma_k, theta_k = self.detect_obstacles(distances)

        optimal_theta_i = self.theta_is[0]
        min_total_potential_field = np.inf
        for theta_i in self.theta_is:
            # Construct repulsive Gaussian potential field
            repulsive_potential_field = self.construct_repulsive_field(
                d_k, sigma_k, theta_k, theta_i)

            # Construct attractive Gaussian potential field
            attractive_potential_field = self.construct_attractive_field(
                theta_i)

            # Total potential field
            total_potential_field = repulsive_potential_field + attractive_potential_field

            # Find the optimal thete that cna minimize the total potential field
            if total_potential_field <= min_total_potential_field:
                optimal_theta_i = theta_i
                min_total_potential_field = total_potential_field

        # Determin the angular z using proportional control
        error = optimal_theta_i - self.yaw_angle
        angular_z = error / self.p

        # Create the msg to publish
        self.obs_avoid_dec_msg.data = [self.linear_x, angular_z]

        # Publishing our vel_msg
        self.obstacle_avoidance_pub.publish(self.obs_avoid_dec_msg)

        # Publish at the desired rate.
        self.rate.sleep()

    def detect_obstacles(self, distance):
        """ Detect all obstacles within the threshold """
        d_k = []
        sigma_k = []
        theta_k = []

        # Init the offset for the angle
        offset = 90

        # If the distance is smaller than the threshold, let it be 0
        distances = np.array(distance)
        distances = np.where(distances < self.threshold, 0, distances)

        # Loop through the distances to find out all obstacles
        start_obstacle = 0
        end_obstacle = 0
        is_obstacle = False
        for i in range(distances.shape[0]):
            if distances[i] == 0:
                if not is_obstacle:
                    start_obstacle = i - offset
                    is_obstacle = True
                    continue

            if distance[i] != 0 and is_obstacle:
                end_obstacle = i - offset

                d_k.append(distances[start_obstacle:end_obstacle].mean())
                sigma_k.append(end_obstacle - start_obstacle)
                theta_k.append(sigma_k[-1] / 2)

                is_obstacle = False

        return d_k, sigma_k, theta_k

    def construct_repulsive_field(self, d_k, sigma_k, theta_k, theta_i):
        """ Construct the repulsive Gaussian potential field """
        f_rep = 0

        for i in range(len(d_k)):
            tilde_d_k = self.d_max - d_k[i]
            A_k = tilde_d_k * np.exp(0.5)
            f = A_k * np.exp(-(theta_k[i] - theta_i)**2 / (2 * sigma_k[i]**2))
            f_rep += f

        return f_rep

    def construct_attractive_field(self, theta_i):
        """ Construct the attractive Gaussian potential field """
        return self.gamma * abs(self.theta_goal - theta_i)


if __name__ == '__main__':
    try:
        obstacle_avoidance_object = TurtleBot(
            node_name='obstacle_avoidance_node')
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
