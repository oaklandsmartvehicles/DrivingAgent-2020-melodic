#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt8
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Bool
"""
#### Actuator Mode
By setting the `twist_mode` argument to false, the Gazebo plugin instead subscribes to four separate actuator command topics so the user can control them directly:

* `/throttle_cmd`: `std_msgs/Float64` topic containing commanded throttle percentage (0 to 1)

* `/brake_cmd`: `std_msgs/Float64` topic containing commanded brake torque in Newton-meters (0 to 1000)

* `/steering_cmd`: `std_msgs/Float64` topic containing commanded steering wheel angle in radians (-9.5 to +9.5)

* `/gear_cmd`: `std_msgs/UInt8` topic containing commanded gear, as controlled by the switch on the dashboard of the real vehicle:
    * 0 = Forward
    * 1 = Reverse
/throttle_cmd std_msg/Float64 0-1 -> /acc std_msgs/Int16  0-255
/brake_cmd std_msg/Float64 0-1 ->  /brake std_msgs/Int16  0-255
/steering_cmd std_msg/Float64 -9.5-9.5 -> /steering std_msgs/Float16  9.5 9.5
/gear_cmd  std_msg/UInt8 0,1  -> /reverse std_msg/Bool true/false
"""
class TopicTf(object):
    def __init__(self):
        self._acc_sub = rospy.Subscriber('throttle_cmd', Float64, self.acc_callback, queue_size=1)
        self._str_sub = rospy.Subscriber('steering_cmd', Float64, self.steering_callback, queue_size=1)
        self._brake_sub = rospy.Subscriber('brake_cmd', Float64, self.brake_callback, queue_size=1)
        self._gear_sub = rospy.Subscriber('gear_cmd', UInt8, self.reverse_callback, queue_size=1)
        self._acc_pub = rospy.Publisher('acc', Int16, queue_size=1)
        self._brake_pub = rospy.Publisher('brake', Int16, queue_size=1)
        self._steering_pub = rospy.Publisher('steering', Float32, queue_size=1)
        self._reverse_pub = rospy.Publisher('reverse', Bool, queue_size=1)

    def acc_callback(self, acc_msg):
        acc = Int16();
        acc.data = int(acc_msg.data * 32. + 180.)
        self._acc_pub.publish(acc)

    def brake_callback(self, brake_msg):
        brake = Int16();
        brake.data = int(brake_msg.data / 100.)
        self._brake_pub.publish(brake)

    def steering_callback(self, steering_msg):
        steering = Float32();
        steering.data = float(steering_msg.data * 180./3.14)
        self._steering_pub.publish(steering)

    def reverse_callback(self, gear_msg):
        reverse = Bool();
        reverse.data = bool(gear_msg.data)
        self._reverse_pub.publish(reverse)



if __name__ == '__main__':
    rospy.init_node('topictf')
    topictf = TopicTf()
    rospy.spin()
