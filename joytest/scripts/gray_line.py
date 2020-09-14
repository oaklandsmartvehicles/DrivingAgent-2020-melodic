#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class GrayExtract(object):
    def __init__(self):
        self._vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self._gray_pub = rospy.Publisher('white_image', Image, queue_size=1)
        self._line_pub = rospy.Publisher('line_data', Int32MultiArray, queue_size=1)
        self._image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.callback)
        self._bridge = CvBridge()
        self._vel = Twist()
        
    def callback(self, data):
        cv_image = self._bridge.imgmsg_to_cv2(data, 'bgr8')
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        line_data = gray_image[399:400,1:640]
 	point = Int32MultiArray();
        point.data = np.sum(line_data,axis = 0)
        gray_image[99:100,1:640]=255
        self._gray_pub.publish(self._bridge.cv2_to_imgmsg(gray_image, 'mono8'))
        self._line_pub.publish(point)

if __name__ == '__main__':
    rospy.init_node('gray_line')
    gray = GrayExtract()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

