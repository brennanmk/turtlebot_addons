#!/usr/bin/env python2.7

'''
Written by Brennan Miller-Klugman 
10/18/22
HRI Interactions Assignment

References:
Created using the locobots provided documentation and source code (Documentation can be found at: https://www.trossen robotics.com/docs/interbotix_xslocobots/ros_packages/)

https://www.ros.org/ was also utilized (used to view message definitions such as the Twist() message used for the /cmd_vel/ topic)
'''

import rospy
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from std_msgs.msg import String
import time
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent, CliffEvent

class test_script:
    def __init__(self):
        rospy.init_node('test_script')

        self.loc_publisher = rospy.Publisher(
            '/move_base/goal', MoveBaseActionGoal, queue_size=3)  # publisher for loc

        self.base_publisher = rospy.Publisher(
            '/mobile_base/commands/velocity', Twist, queue_size=3)  # publisher for base movement

        self.affect_publisher = rospy.Publisher(
            '/affect', String, queue_size=3)  # publisher for base movement
        time.sleep(5)
        self.affect_publisher.publish("happy")

        self.move_to_loc(0.691, 0.692, 0.000, 0.000, 0.000, 0.969, 0.245) #move to start location
        rospy.wait_for_message('/move_base/result', MoveBaseActionResult) #wait for confirmation that the locobot has moved to the correct pos
        temp = raw_input("Press enter to continue")

        self.affect_publisher.publish("confused")
        self.control_base(0,0.4,20) #spin for 10 seconds
        temp = raw_input("Press enter to continue")

        self.affect_publisher.publish("happy")

        self.move_to_loc(-0.481, 0.794, 0.000, 0.000, 0.000, 0.960, 0.279) #move to start location
        rospy.wait_for_message('/move_base/result', MoveBaseActionResult) #wait for confirmation that the locobot has moved to the correct pos
        self.affect_publisher.publish("tired")
        temp = raw_input("Press enter to continue")
        
        self.affect_publisher.publish("happy")

        self.move_to_loc(-0.202, 3.476, 0.000, 0.000, 0.000, 0.480, 0.877) #move to start location
        rospy.wait_for_message('/move_base/result', MoveBaseActionResult) #wait for confirmation that the locobot has moved to the correct pos
        self.affect_publisher.publish("sad")
        temp = raw_input("Press enter to continue")

        self.affect_publisher.publish("angry")
        temp = raw_input("Press enter to continue")
        self.affect_publisher.publish("happy")

        rospy.spin()

    def control_base(self, x, yaw, duration):
        '''function to control base, takes x linear and yaw as inputs. 
            Base moves as a velocity in m/s for the duration specified, 
            there is a similair function in https://github.com/Interbotix/interbotix_ros_toolboxes/blob/main/interbotix_xs_toolbox/interbotix_xs_modules/src/interbotix_xs_modules/locobot.py, 
            this function was written to gain a better understanding of how the geometry/Twist message type works.'''
        msg = Twist()
        msg.angular.y = 0.0
        msg.angular.y = 0.0
        msg.angular.z = yaw
        msg.linear.x = x
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        counter = 0  # counter for time
        while counter < duration:  # publish a cmd_vel every 0.05s for duration entered
            self.base_publisher.publish(msg)
            time.sleep(0.1)
            counter = counter + 0.1


    def move_to_loc(self, pos_x, pos_y, pos_z , orient_x, orient_y, orient_z, orient_w):
        '''function to set a goal location'''

        msg = MoveBaseActionGoal()
        msg.goal.target_pose.header.frame_id='map'
        msg.goal.target_pose.pose.position.x = pos_x
        msg.goal.target_pose.pose.position.y = pos_y
        msg.goal.target_pose.pose.position.z = pos_z
        msg.goal.target_pose.pose.orientation.z = orient_z
        msg.goal.target_pose.pose.orientation.w = orient_w
        self.loc_publisher.publish(msg)
    
if __name__ == '__main__':
    test_script()
