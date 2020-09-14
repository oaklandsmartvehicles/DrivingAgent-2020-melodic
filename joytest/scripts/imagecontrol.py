#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from PyQt5.QtWidgets import QApplication, QMainWindow, QMenu, QVBoxLayout, QSizePolicy, QMessageBox, QWidget, QPushButton
from PyQt5.QtGui import QIcon

import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from std_msgs.msg import UInt8
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension
from PyQt5.QtWidgets import QApplication, QMainWindow, QMenu, QVBoxLayout, QSizePolicy, QMessageBox, QWidget, QPushButton
from PyQt5.QtGui import QIcon
def indices(a, func):
    return [i for (i,val) in enumerate(a) if func(val)]

class ImageControl():
    def __init__(self):
        self._image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.callback)
        self._gray_pub = rospy.Publisher('white_image', Image, queue_size=1)
        self._line1_pub = rospy.Publisher('line1_data', Int32MultiArray, queue_size=1)
        self._line2_pub = rospy.Publisher('line2_data', Int32MultiArray, queue_size=1)
        self._line3_pub = rospy.Publisher('line3_data', Int32MultiArray, queue_size=1)
        self._line4_pub = rospy.Publisher('line4_data', Int32MultiArray, queue_size=1)

        self._csteeringline_pub = rospy.Publisher('c_steeringline', Float64, queue_size=1)
        self._cstopline_pub = rospy.Publisher('c_stopline', Bool, queue_size=1)
        self._bridge = CvBridge()
        self._csteeringline = Float64()
        self._cstopline = Bool()
  
    def callback(self, data):
        cv_image = self._bridge.imgmsg_to_cv2(data, 'bgr8')
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        gray_image = gray_image > np.max(np.max(gray_image))
        line1_data = gray_image[120:122,1:640]
       
        line2_data = gray_image[240:242,1:640]
        line3_data = gray_image[1:480,480:482]
        line4_data = gray_image[1:480,320:322]
 	line1 = Int32MultiArray();
 	line2 = Int32MultiArray();
 	line3 = Int32MultiArray();
 	line4 = Int32MultiArray();

        line1.data = np.sum(line1_data,axis = 0)
        line2.data = np.sum(line2_data,axis = 0)
        line3.data = np.sum(line3_data,axis = 0)
        line4.data = np.sum(line4_data,axis = 0)
 #       tmpline = np.asarray(line1.data[1:638],dtype=float)-np.asarray(line1.data[2:639],dtype=float)
#        inds = indices(np.abs(tmpline),lambda x: x > 20)
        print np.array(line1.data,dtype=float) > np.max(line1.data)*0.8


        gray_image[120:122,1:640]=255
        gray_image[240:242,1:640]=255
        gray_image[1:480,480:482]=255
        gray_image[1:480,320:322]=255

        self._gray_pub.publish(self._bridge.cv2_to_imgmsg(gray_image, 'mono8'))
        self._line1_pub.publish(line1)
        self._line2_pub.publish(line2)
        self._line3_pub.publish(line3)
        self._line4_pub.publish(line4)

        self._csteeringline.data = 0.0
        self._csteeringline_pub.publish(self._csteeringline)
        self._cstopline.data = False
        self._cstopline_pub.publish(self._cstopline)


if __name__ == '__main__':

    rospy.init_node('imagecontrol')
    imagecontrol = ImageControl()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

