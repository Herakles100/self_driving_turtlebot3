#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import roslaunch
from line_following_simulation import LineFollower
from move_TurtleBot3 import MoveTurtleBot3



class NodeController:
    def __init__(self):
    	
    	self.stop_flag = rospy.Subscriber("/stop_sign_flag", Bool, self.launch_node)
    	self.line_vel = rospy.Subscriber("/line/cmd_vel", Twist, self.launch_node)

    def launch_node(self, data):
    	if data == False:
    		rospy.init_node('')

def main():
    rospy.init_node('main_node', anonymous=True)
    node_controller_object = NodeController()
    rate = rospy.Rate(5)
    ctrl_c = False

    def shutdownhook():
        # Works better than rospy.is_shutdown()
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)
    while not ctrl_c:
        rate.sleep()


if __name__ == '__main__':
    main()
