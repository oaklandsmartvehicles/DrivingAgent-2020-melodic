#!/usr/bin/env python
import rospy
import socket
import struct
import threading
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy

UDP_IP = "192.168.2.100"
UDP_PORT = 12090

sock = socket.socket(socket.AF_INET, # Internet
             socket.SOCK_DGRAM) # UDP

class transmitter(threading.Thread):

    def __init__(self, i):
        threading.Thread.__init__(self)
        self.boolean_commands = 0
        self.vehicle_speed = 0
        self.steering_angle = 0x7FFF
        self.brake = 0
        self.h = i

    def run(self):
        rate = rospy.Rate(100) # 100hz
        while not rospy.is_shutdown():
            UpdateUDPData()
            SendUDPData()
            rate.sleep()

transmit_thread = transmitter(1)


#To be used when developing driving agent to publish speed 
def vehicle_speed_callback(data):
    transmit_thread.vehicle_speed = int(data.data)

#To be used when developing driving agent to publish steering angle
def steering_angle_callback(data):
    transmit_thread.steering_angle = int(data.data)

#To be used when developing driving agent to publsih brake command
def brake_callback(data):
    transmit_thread.brake = int(data.data)

#To be used to enable or disable tele-operation mode
def tele_op_callback(data):
    if data.data:      
        transmit_thread.boolean_commands |= 0x10
    else:
        transmit_thread.boolean_commands &= ~0x10 

def joy_callback(data):
    #print(data.axes[5])
    vehicle_speed = data.axes[5]
    vehicle_speed *= -1
    vehicle_speed += 1
    vehicle_speed *= 0x7FFF
    print(vehicle_speed)
    transmit_thread.vehicle_speed = vehicle_speed
    #print(data.axes[0])
    transmit_thread.steering_angle = (data.axes[0] * 0x7FFF) + 0x7FFF
    print(transmit_thread.steering_angle)
    
    if data.buttons[0]:      
        transmit_thread.boolean_commands |= 0x10
    else:
        transmit_thread.boolean_commands &= ~0x10 
    


def UpdateUDPData():
    #don't forget to update the bytearray size below
    struct.pack_into('<H', ba, 0, transmit_thread.boolean_commands)
    struct.pack_into('<H', ba, 2, transmit_thread.vehicle_speed)
    struct.pack_into('<H', ba, 4, transmit_thread.steering_angle)
    struct.pack_into('<H', ba, 6, transmit_thread.brake)

def SendUDPData():
    sock.sendto(ba, (UDP_IP, UDP_PORT))

ba = bytearray(30) #this needs to match the size of the data struct packed above

def main():

    
    rospy.Subscriber("/joystick/accel", Float32, vehicle_speed_callback)
    rospy.Subscriber("/joystick/steering", Float32, steering_angle_callback)
    rospy.Subscriber("/joystick/brake", Float32, brake_callback)
    rospy.Subscriber("/joystick/tele_op", Bool, tele_op_callback)
    rospy.Subscriber("/joy", Joy, joy_callback)
    rospy.init_node('middleman', anonymous=True)
    transmit_thread.start()

if __name__ == '__main__':
    try:
        main()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
