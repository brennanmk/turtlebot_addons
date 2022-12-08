# Brennan Miller-Klugman
# 12/07/22
# This script is used to monitor CPU and ram usage of the laptop controlling the turtlebot

# Referances:
# 


http://docs.ros.org/en/hydro/api/kobuki_msgs/html/msg/PowerSystemEvent.html
http://docs.ros.org/en/hydro/api/kobuki_msgs/html/msg/RobotStateEvent.html
http://docs.ros.org/en/hydro/api/kobuki_msgs/html/msg/CliffEvent.html

diagnostic_msgs/DiagnosticArray #can be used to get laptop charge and base charge
"Laptop Battery"


"mobile_base_nodelet_manager: Battery"
    values:
      -
        key: "Voltage (V)"
        value: "16"
      -
        key: "Percent"
        value: "85.6061"
#POTENTIALLY USEFUL
http://docs.ros.org/en/hydro/api/kobuki_msgs/html/msg/MotorPower.html
http://docs.ros.org/en/hydro/api/kobuki_msgs/html/msg/ButtonEvent.html

import rospy
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseActionResult
from kobuki_msgs.msg import BumperEvent # http://docs.ros.org/en/hydro/api/kobuki_msgs/html/msg/BumperEvent.html
from std_msgs.msg import Float32

class affectual_mapping:
    def __init__(self):
        
        self.affects = {"bump":"sad", "battery_low": "tired"} # TODO : complete a dictionary mapping

        rospy.init_node('affectual_mapping') #initalize ros node

        self.robot_name = 'locobot' # TODO:Use argparse https://docs.python.org/3/library/argparse.html (Or ros parameter server) to get robot name

        self.affect_publisher = rospy.Publisher(
            f'/{self.robot_name}/affect', String, queue_size=3)  # publisher for robot affect, this is what is taken as input in Unity

        rospy.Subscriber(f'/{self.robot_name}/battery', BATTERYMESSAGE TYPE, self.battery)  # subscriber for battery level, every time a new battery message is recieved, the battery function is called #TODO: find what type the battery message is
               
        rospy.Subscriber(f'/{self.robot_name}/move_base/result', MoveBaseActionResult, self.goal_tracker)  # subscriber for hazard detection (bump sensor)

        rospy.Subscriber(f'/{self.robot_name}/bumper', BumperEvent, self.hazzard)  # subscriber for hazard detection
        
        rospy.Subscriber(f'/{self.robot_name}/cpu_usage', Float32, self.cpu)  # subscriber for hazard detection

        rospy.Subscriber(f'/{self.robot_name}/memory_usage', Float32, self.memory)  # subscriber for hazard detection

        rospy.spin()

    def hazzard(self, data):
        if data.state == 1:
            self.affect_publisher.publish(self.affects["bump"])
    
    def cpu(self, data):
        if data.data > 70:
            self.affect_publisher.publish(self.affects["bump"])
    
    def memory(self, data):
        if data.data > 70:
            self.affect_publisher.publish(self.affects["bump"])
    
    def battery(self, data):
        if data.battery_level <= 20: #if battery is below some threshold, publish tired affect... could also have multiple stages of tiredness
            self.affect_publisher.publish(self.affects["battery_low"])

    def goal_tracker(self, data):
        if data.status.status == 4 or data.status.status == 5: #if goal is not reached, publish sad affect
            self.affect_publisher.publish("confused")

if __name__ == '__main__':
    affectual_mapping()
