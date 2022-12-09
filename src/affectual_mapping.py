#!/usr/bin/env python2.7

# Brennan Miller-Klugman
# 12/07/22
# This script is used to listen to messages published by the robot and publish affect messages 

# Referances:
# http://docs.ros.org/
# https://www.programiz.com/python-programming/nested-dictionary

import rospy
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseActionResult
from kobuki_msgs.msg import BumperEvent, CliffEvent
from std_msgs.msg import Float32
from diagnostic_msgs.msg import DiagnosticArray


class affectual_mapping:
    def __init__(self):
        self.event_monitor = {
                "bump" : {"status": False, "time": 0, "priority": 0},
                "cliff" : {"status": False, "time": 0, "priority": 1},
                "laptop_battery" : {"status": False, "time": 0, "priority": 3},
                "base_battery" : {"status": False, "time": 0, "priority": 4},
                "cpu" : {"status": False, "time": 0, "priority": 5},
                "memory" : {"status": False, "time": 0, "priority": 6},
                "goal" : {"status": False, "time": 0, "priority": 2}
            }
        
        self.last_published_affect = ""

        self.affects = {"bump": "sad", "battery_low": "tired"}

        rospy.init_node('affectual_mapping')

        self.robot_name = rospy.get_param("robot_name")

        self.affect_publisher = rospy.Publisher(
            f'/{self.robot_name}/affect', String, queue_size=3) 

        rospy.Subscriber('/laptop_battery_percentage',
                         Float32, self.base_battery)  # subscriber for laptop battery

        rospy.Subscriber('/cpu_usage', Float32, self.cpu)

        rospy.Subscriber('/memory_usage',
                         Float32, self.memory)

        # subscriber for diagnostic messages, used to get base battery level
        rospy.Subscriber('/diagnostics',
                         DiagnosticArray, self.diagnostic)

        rospy.Subscriber('/move_base/result',
                         MoveBaseActionResult, self.goal_tracker)

        rospy.Subscriber('/mobile_base/events/bumper',
                         BumperEvent, self.bumper)

        rospy.Subscriber('/{mobile_base/events/cliff', CliffEvent, self.cliff)

        # call Query function every 5 seconds
        rospy.Timer(rospy.Duration(1), self.query)

        rospy.spin()

    def bumper(self, data):
        if data.state == 1:
            self.event_monitor["bump"]["status"] = True
            self.event_monitor["bump"]["time"] = rospy.get_time()

    def cliff(self, data):
        if data.state == 1:
            self.event_monitor["cliff"]["status"] = True
            self.event_monitor["cliff"]["time"] = rospy.get_time()

    def cpu(self, data):
        if data.data > 70:
            self.event_monitor["cpu"]["status"] = True
            self.event_monitor["cpu"]["time"] = rospy.get_time()

    def memory(self, data):
        if data.data > 70:
            self.event_monitor["memory"]["status"] = True
            self.event_monitor["memory"]["time"] = rospy.get_time()

    def goal_tracker(self, data):
        if data.status.status == 4 or data.status.status == 5:  # if goal is not reached, publish sad affect
            self.event_monitor["goal"]["status"] = True
            self.event_monitor["goal"]["time"] = rospy.get_time()
    
    def diagnostic(self, data):  # Use diagnostic messages to check kobuki battery level
        for val in data.status:
            if val.name == "mobile_base_nodelet_manager: Battery":
                if val.values[2].value <= 20:
            self.event_monitor["base_battery"]["status"] = True
            self.event_monitor["base_battery"]["time"] = rospy.get_time()

    def laptop_battery(self, data):
        if data.data <= 20:
            self.event_monitor["laptop_battery"]["status"] = True
            self.event_monitor["laptop_battery"]["time"] = rospy.get_time()

    def query(self, data):
        affect_to_display = ""
        for key, value in self.event_monitor.items():
            if value["status"] == True: 
                if rospy.get_time().secs - value["time"].secs > 5: # reset event if it has been triggered for more than 5 seconds
                    value["status"] = False
                    value["time"] = 0
                else:
                    if affect_to_display == "":
                        affect_to_display = key
                    elif self.event_monitor[affect_to_display]["priority"] > value["priority"]:
                        affect_to_display = key

        if(self.last_published_affect != self.affects[key]): # If affect has changed, publish new affect
            self.affect_publisher.publish(self.affects[key])
            self.last_published_affect = self.affects[key]

if __name__ == '__main__':
    affectual_mapping()
