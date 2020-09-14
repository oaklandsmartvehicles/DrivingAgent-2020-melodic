#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan 
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from std_msgs.msg import Bool
class ScanObj(object):
    def __init__(self):
        self._scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback, queue_size=1)
        self._obj_pub = rospy.Publisher('obj', Float64, queue_size=1)


    def scan_callback(self, scan_msg):
        range = Float64();
        range.data = scan_msg.ranges[540]
        self._obj_pub.publish(range)
           

if __name__ == '__main__':
    rospy.init_node('scan2obj')
    scan2obj = ScanObj()
    rospy.spin()
    
