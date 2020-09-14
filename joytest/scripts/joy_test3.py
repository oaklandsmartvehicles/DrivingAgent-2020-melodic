#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import Bool
class JoyTwist(object):
    def __init__(self):
        self._joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback, queue_size=1)
        self._acc_pub = rospy.Publisher('acc', Int16, queue_size=1)
        self._str_pub = rospy.Publisher('steering', Float32, queue_size=1)
        self._brake_pub = rospy.Publisher('brake', Int16, queue_size=1)
        self._rev_pub = rospy.Publisher('reverse',Bool, queue_size=1)

    def joy_callback(self, joy_msg):
        if joy_msg.axes[5] < -0.9:
            acc = Int16();
            str = Float32();
            brake = Int16();
            rev = Bool();
            joyacc = joy_msg.axes[1]
            joystr = joy_msg.axes[3]
            joyrev = joy_msg.buttons[0]
            joyfor = joy_msg.buttons[3]
            if joyacc >= 0:
                acc.data = joyacc * 39.0 + 180.0
                self._acc_pub.publish(acc)
            else:
                if joyacc < -0.8:
                    brake.data = 170
                    self._brake_pub.publish(brake)
                else:
                    if joyrev == 1:
                         rev.data = True
                         self._rev_pub.publish(rev)
                         acc.data = 210
                         self._acc_pub.publish(acc)
                    else:
                        if joyfor == 1:
                            rev.data = False
                            self._rev_pub.publish(rev)
                            acc.data = 203
                            self._acc_pub.publish(acc)
                        else:
                            rev.data = False
                            self._rev_pub.publish(rev)
                            acc.data = 0
                            self._acc_pub.publish(acc)

            str.data = joy_msg.axes[3] * 20.0
            self._str_pub.publish(str)


if __name__ == '__main__':
    rospy.init_node('joy_twist')
    joy_twist = JoyTwist()
    rospy.spin()
    
