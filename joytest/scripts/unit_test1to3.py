#!/usr/bin/env python


# roslaunch joytest unit_test1to3.py my_args="1 30"
# speed 1[m/s] distance 30[m]
from __future__ import print_function
import rospy
from std_msgs.msg import UInt8
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import sys
import time

class UnitTest(object):
    def __init__(self,spd=1,distance=5):
        self._spd = float(spd)
        self._distance = float(distance)
        self._c_speed_sub = rospy.Subscriber('c_speed', Float32, self.c_speed_callback, queue_size=1)
        self._obj_sub = rospy.Subscriber('obj', Float64, self.obj_callback, queue_size=1)
        self._e_stop_sub = rospy.Subscriber('c_estop', Bool, self.c_estop_callback, queue_size=1)
        self._accdist_pub = rospy.Publisher('accdist',Float32,queue_size=1)
        self._cmdvel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self._rlight_pub = rospy.Publisher('rlight', Int16, queue_size=1)
        self._start = time.time()
        self._obj = 30
        self._c_estop = False
        self._accdist = 0.0
        
    def c_speed_callback(self, c_speed_msg):
        cmd_vel = Twist();
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = 0.0
        time1 = time.time()j
        elapsed_time = time1 - self._start
        self._start = time1
        self._accdist = self._accdist + c_speed_msg.data * elapsed_time;
        rlight = Int16();
        if self._accdist > self._distance:
            cmd_vel.linear.x = 0.0
            rlight.data = 1
        else:
            cmd_vel.linear.x = self._spd
            rlight.data = 2
        if self._obj < 1.0:
            cmd_vel.linear.x = 0.0
            rlight.data = 2
        if self._c_estop == True:
            cmd_vel.linear.x = 0.0
            rlight.data = 1
        self._cmdvel_pub.publish(cmd_vel)
        self._rlight_pub.publish(rlight)
        accdist = Float32();
        accdist.data = self._accdist
        self._accdist_pub.publish(accdist)
    def obj_callback(self, obj_msg):
        self._obj = obj_msg.data
    def c_estop_callback(self, c_estop_msg):
        self._c_estop = c_estop_msg.data
if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("usage: unit_test1to3.py speed[m/s] moving_distance[m]")
        print("usage: roslaunch joytest unit_test1to3.py my_args=\"1 5\" ")
    else:
        rospy.init_node('unittest')
        unittest = UnitTest(sys.argv[1],sys.argv[2])
        rospy.spin()
