# Brennan Miller-Klugman
# 12/07/22
# This script is used to monitor battery life, and CPU + ram usage of a computer using psutil

# Referances:
# https://www.geeksforgeeks.org/how-to-get-current-cpu-and-ram-usage-in-python/
# https://www.geeksforgeeks.org/python-script-to-shows-laptop-battery-percentage/
# https://psutil.readthedocs.io/en/latest/index.html?highlight=temperature

import rospy
from std_msgs.msg import Float32
import psutil

class system_monitor:
    def __init__(self):
        rospy.init_node('system_monitor')

        self.cpu_pub = rospy.Publisher('/cpu_usage', Float32, queue_size=3) 
        self.memory_pub = rospy.Publisher('/memory_usage', Float32, queue_size=3) 
        self.battery_pub = rospy.Publisher('/laptop_battery_percentage', Float32, queue_size=3) 
        rospy.Timer(rospy.Duration(5), self.query) # call Query function every 5 seconds
        rospy.spin()

    def query(self, data): 
        # Query function uses psutil to get battery, cpu usage, and cpu temp data and subsequently publishes the data

        #Use psutil to get laptop information
        cpu_usage = psutil.cpu_percent(4)
        memory_usage = psutil.virtual_memory()[3]/1000000000 #Memory usage in GB
        battery = psutil.sensors_battery().percent 

        #publish information
        self.cpu_pub.publish(cpu_usage)
        self.memory_pub.publish(memory_usage)
        self.battery_pub.publish(battery)

if __name__ == '__main__':
    system_monitor()
