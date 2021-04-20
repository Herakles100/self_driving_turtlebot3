#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray


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

        # Subscriber to subscribe to '/scan' topic
        self.laser_sub = rospy.Subscriber(
            'scan', LaserScan, self.laser_callback)

        self.obs_avoid_dec_msg = Float32MultiArray()

        # Default velocity
        self.linear_x = rospy.get_param('~linear_x')

        # Init view range
        self.view_range = eval(rospy.get_param('~view_range'))

        # Init the publish rate
        self.rate = rospy.Rate(10)

        # Init distance reading
        self.current_distance = [100] * 360

    def laser_callback(self, laser_data):
        # Update only if the incoming range data is within the limits
        for i, dist in enumerate(laser_data.ranges):
            if laser_data.range_min < dist < laser_data.range_max:
                self.current_distance[i] = dist

        # Keep moving till the robot sees an obstacle at
        # less than safe distance
        self.obs_avoid_dec_msg.data = self.make_decision()

        # Publishing our vel_msg
        self.obstacle_avoidance_pub.publish(self.obs_avoid_dec_msg)

        # Publish at the desired rate.
        self.rate.sleep()

    def make_decision(self):
        # Segmenting the laser data
        # straight ahead
        fwd = self.current_distance[0]
        # forward view
        ahead = self.current_distance[-20:-1] + self.current_distance[:20]
        
        #37
        
        # Left view
        left = self.current_distance[-self.view_range[1]:-20]
        # Right view
        right = self.current_distance[20:self.view_range[1]]
        
        front_right = self.current_distance[10:self.view_range[1]-10]
        front_left = self.current_distance[-self.view_range[1]+10:-10]
        
        # Greater range on the higher end gives you a better search space
        # Greater range on the lower end could confuse the robot
        # Too great of range on the higher end can also confuse the robot
        
        # 15
        
        # Total_range = self.current_distance[-self.view_range[1]:self.view_range[1]]
        # Take average
        
        
        # this crops the search space so we don't consider distances that are too far ahead
        # front_right = [x for x in front_right if x < .9]
        # front_left = [x for x in front_left if x < .9]
        
        ahead_mean = np.mean(ahead)
        left_mean = np.mean(left)
        right_mean = np.mean(right)
        
        front_right_mean = np.mean(front_right)
        front_left_mean = np.mean(front_left)
        
        # Mean helps us find the larger gaps
        # Min helps you find the gap with the farthest next obstacle and tells you how far the closest object is
        # Max helps you find the gap with the farthest distance and tells you how far that object is
        
        


        # Pick the minimum distance in the forward view
        ahead_min = np.min(ahead)
        left_min = np.min(left)
        right_min = np.min(right)
        
        rospy.logwarn('--------------------------------------------')
        rospy.logwarn('Ahead: ' + str(ahead_min))
        rospy.logwarn('Front right: ' + str(front_right_mean))
        rospy.logwarn('Front left: ' + str(front_left_mean))
        rospy.logwarn('--------------------------------------------')
        

        # Setting up the Proportional gain values
        K_left = left_mean / (ahead_mean + right_mean)
        K_right = right_mean / (ahead_mean + left_mean)
        

        
        K_ahead = ahead_min

        # Checking the distance from obstacles and
        # controlling speeds accordingly
        angular_z = K_right - K_left
        linear_x = min(K_ahead * self.linear_x, 0.1)

        # Take the below policy if not in Gazebo
        
        # Too high of an angular velocity will miss certain windows of opportunity
        # .5 is too high
        # .42 is too low
        
        # Too low of an angular velocity and you won't be able to perform the maneuveur
        
        if self.work_mode != 'simulation':
        
            # Changing the higher end will make the bot decide which way to look for an opening
            if 0.1 <= ahead_min <= 0.6 or 0.1 <= left_min <= 0.2 or 0.1 <= right_min <= 0.2:
                if front_right_mean >= front_left_mean:
                    angular_z = .50
                    
                else:
                    angular_z = -.50
            elif ahead_min < 0.1:
                linear_x = 0

        return [linear_x, angular_z, ahead_min]


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