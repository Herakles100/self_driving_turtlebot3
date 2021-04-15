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

        lidar_range = self.current_distance[-90:90]
        
        threshold = 2
        
        Phi_k = 2*np.atan2(lidar_range_k*tan((-90+k)/2) + .178/2, lidar_range_k)
        
        d_max = 3.5
        
        d_bar_k = d_max - lidar_range_k
        
        A_k = d_bar_k*np.exp(1/2)
        sigma_k= (-90+k)/2
        
        f_rep = sum(A_k*np.exp(-((theta_k - theta_i)^2)/(2*sigma_k^2)))
        
        f_att = 5*abs(theat_goal - theta_i)
        
        f_total = f_rep + f_att
        
        angle = np.argmin(f_total)
        
        # Rotate the bot to that angle
        
        linear_x = self.linear_x
        angular_velocity = 0.5
        
	current_angle = 0
	t0 = rospy.Time.now().to_sec()

	if current_angle < angle:
	    # Calculate the current distance
	    t1 = rospy.Time.now().to_sec()
	    angular_z = angular_velocity
	    current_angle = angular_z * (t1 - t0)
	    # Publish at the desired rate.
	    self.rate.sleep()
	else:
	    angular_z = 0

        return [linear_x, angular_z, fwd]


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
