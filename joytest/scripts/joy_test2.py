#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16
from std_msgs.msg import Float32

class JoyTwist(object):
    def __init__(self):
        self._joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback, queue_size=1)
        self._acc_pub = rospy.Publisher('acc', Int16, queue_size=1)
        self._str_pub = rospy.Publisher('steering', Float32, queue_size=1)
        self._brake_pub = rospy.Publisher('brake', Int16, queue_size=1)

    def joy_callback(self, joy_msg):
        if joy_msg.buttons[0] == 1:
            acc = Int16();
            str = Float32();
            brake = Int16();
            joyacc = joy_msg.axes[1]
            joystr = joy_msg.axes[0]
            if joyacc >= 0:
                acc.data = joyacc * 35.0 + 180.0
                self._acc_pub.publish(acc)
            else:
                if joyacc < -0.8:
                    brake.data = 170
                    self._brake_pub.publish(brake)
            str.data = joy_msg.axes[0] * 10.0
            self._str_pub.publish(str)


if __name__ == '__main__':
    rospy.init_node('joy_twist')
    joy_twist = JoyTwist()
    rospy.spin()
    
