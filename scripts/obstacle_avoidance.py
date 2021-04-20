#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# TODO: IMPROVE PERFORMANCE--NOT CONSISTENT
# Read my notes in the script for guidance

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
        
        
        self.current_yaw = 0
        
        self.pose = rospy.Subscriber('/odom', Odometry, get_rotation)
        

        self.obs_avoid_dec_msg = Float32MultiArray()

        # Default velocity
        self.linear_x = rospy.get_param('~linear_x')
        
        self.angular_z = rospy.get_param('~angular_z')
        
        # Default L-R
        self.last = ""
        
        self.gap_dist_threshold = 0.1
        self.gap_size_threshold = 0.25

        # Init view range
        self.view_range = eval(rospy.get_param('~view_range'))

        # Init the publish rate
        self.rate = rospy.Rate(10)

        # Init distance reading
        self.current_distance = [100] * 360
        
        self.pre_turn = {"yaw_angle":0, "time":0, "center":0, "angular_z":0}
    
    
    
   def get_rotation(msg):
       orientation_q = msg.pose.pose.orientation
       orientation_list [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
       *_, self.current_yaw = euler_from_quaternion(orientation_list)
    
   def get_gap_size(self, distance, shift):
        """ Detect all obstacles within the threshold """
        sigma_k = []
        theta_k = []


        # If the distance is smaller than the threshold, let it be 0
        distances = np.array(distance)
        distances = np.where(distances > np.min(distances) + self.gap_dist_threshold, distances, 0)

        # Loop through the distances to find out all obstacles
        start_gap = 0
        end_gap = 0
        is_gap = False
        for i in range(distances.shape[0]):
            if distances[i] != 0:
                if not is_gap:
                    start_gap = i
                    is_gap = True
                    continue

            else:
                if is_gap:
                    end_gap= i

                    sigma_k.append(end_gap - start_gap)
                    theta_k.append(sigma_k[-1] / 2 - shift)

                    # Reset the flag
                    is_gap = False
                        
        max_sigma = np.max(sigma_k)
        max_theta = theta_k[np.argmax(sigma_k)]

        return max_sigma, max_theta
        
    def get_preference_flag(self, last):
        # Store previous turn and use it to determine next turn
        if last == "Left"
            preference = False
        else:
            preference = True
        return preference

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
        ahead = self.current_distance[-15:-1] + self.current_distance[:15]
        
        
        # Left view
        left = self.current_distance[-90:-15]
        # Right view
        right = self.current_distance[15:90]
        
        front_right = self.current_distance[10:self.view_range[1]-10]
        front_left = self.current_distance[-self.view_range[1]+10:-10]
        
        # Greater range on the higher end gives you a better search space
        # Greater range on the lower end could confuse the robot
        # Too great of range on the higher end can also confuse the robot
        
        # Total_range = self.current_distance[-self.view_range[1]:self.view_range[1]]
        # Take average
        
        
        # this crops the search space so we don't consider distances that are too far ahead
        #front_right = [x for x in front_right if x < .9]
        #front_left = [x for x in front_left if x < .9]
        
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
        
        
        # Setting up the Proportional gain values
        K_left = left_mean / (ahead_mean + right_mean)
        K_right = right_mean / (ahead_mean + left_mean)
        
        K_front_right = front_right_mean / (front_left_mean)
        K_front_left = front_left_mean / (front_right_mean)
        
        K_ahead = ahead_min
        
        
        gap_dist_threshold = self.gap_dist_threshold
        gap_size_threshold = self.gap_size_threshold
        
        

            
        ahead_gap_size, ahead_gap_center = self.get_gap_size(ahead,15)
        right_gap_size, right_gap_center = self.get_gap_size(right,90)
        left_gap_size,  left_gap_center = self.get_gap_size(left,90)
        
        if self.pre_turn["yaw_angle"] == 0:
            if ahead_min >= gap_dist_threshold:
                K_ahead = ahead_min
                linear_x = K_ahead * self.linear_x
                center = ahead_gap_center
                time = center/self.angular_z
            else:
                if left_min >= gap_dist_threshold:
                    # Turn left if you have not last turned right
                    preference = self.get_preference_flag(self.last)
                    if right_min >= gap_dist_threshold:
                        if preference == True:
                            # Turn Left
                            center = left_gap_center
                            time = center/self.angular_z
                            self.last = "Left"
                        else:
                            # Turn right
                            center = right_gap_center
                            time = center/self.angular_z
                            self.last = "Right"
                    else:
                        # turn Left
                        center = left_gap_center
                        time = center/self.angular_z
                        self.last = "Left"
                    
                 elif left_min < gap_dist_threshold:
                    if right_min >= gap_dist_threshold:
                       # Turn right
                       center = right_gap_center
                       time = center/self.angular_z
                       self.last = "Right"
                    elif right_min < gap_dist_threshold:
                        if ahead_gap_size >= gap_size_threshold:
                            K_ahead = ahead_min
                            linear_x = K_ahead * self.linear_x
                            center = ahead_gap_center
                            time = center/self.angular_z
                        else:
                            if left_gap_size >= gap_size_threshold:
                                preference = self.get_preference_flag(self.last)
                                if right_gap_size >= gap_size_threshold:
                                    if preference == True:
                                        center = left_gap_center
                                        time = center/self.angular_z
                                        self.last = "Left"
                                    else:
                                        # Turn right
                                        center = right_gap_center
                                        time = center/self.angular_z
                                        self.last = "Right"
                                else:
                                    # turn Left
                                    center = left_gap_center
                                    time = center/self.angular_z
                                    self.last = "Left"
                            elif left_gap_size < gap_size_threshold:
                                if right_min >= gap_size_threshold:
                                    center = ahead_gap_center
                                    time = center/self.angular_z
                                    self.last = "right"
                                else:
                                    linear_x = 0
                                    angular_z = 0
            try: 
                if center > 0:
                    angular_z = -self.angular_z
                else:
                    angular_z = self.angular_z
            except:
                linear_x = 0
                angular_z = 0
            
            self.pre_turn["yaw_angle"] = self.current_yaw
            self.pre_turn["center"] = center
            self.pre_turn["time"] = time
            self.pre_turn ["angular_z"] = angular_z  
                             
        elif abs(self.current_yaw -  self.pre_turn["yaw_angle"]) >= self.pre_turn["center"]:
            self.pre_turn["yaw_angle"] = 0
            self.pre_turn["center"] = 0
            self.pre_turn["time"] = 0
            self.pre_turn["angular_z"] = 0
        
        
        elif abs(self.current_yaw -  self.pre_turn["yaw_angle"]) < self.pre_turn["center"]         
            linear_x = K_ahead * self.linear_x
            angular_z = self.pre_turn["angular_z"]
            center = self.pre_turn["center"]
            time = self.pre_turn["time"]               

                
        
        #rospy.logwarn('--------------------------------------------')
        #rospy.logwarn('Ahead: ' + str(ahead_min))
        #rospy.logwarn('Front right: ' + str(front_right_mean))
        #rospy.logwarn('Front left: ' + str(front_left_mean))
        #rospy.logwarn('--------------------------------------------')
        

        # Checking the distance from obstacles and
        # controlling speeds accordingly
        #angular_z = K_right - K_left
        #linear_x = K_ahead * self.linear_x

        # Take the below policy if not in Gazebo
        
        # Too high of an angular velocity will miss certain windows of opportunity
        # .5 is too high
        # .42 is too low
        
        # Too low of an angular velocity and you won't be able to perform the maneuveur
        
       # if self.work_mode != 'simulation':
       # 
       #     # Changing the higher end will make the bot decide which way to look for an opening
       #     if 0.1 <= ahead_min <= 0.6 or left_min <= 0.25 or right_min <= 0.25:
       #         #angular_z = (K_front_right - K_front_left) # might multiply a constant
       #         if front_right_mean >= front_left_mean:
       #             angular_z = .55
       #             # .55
       #             
       #         else:
       #             angular_z = -.55
       #         #    # -.55
       #     elif ahead_min < 0.1:
       #         linear_x = 0

        return [linear_x, angular_z, ahead_min, time, center]


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
