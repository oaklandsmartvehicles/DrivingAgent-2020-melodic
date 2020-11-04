#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <stdio.h>

namespace Joystick
{
  class joystick
  {

    public:
    joystick(ros::NodeHandle& n, ros::NodeHandle& pn);
  

  private:
    void recvjoydata(const sensor_msgs::Joy::ConstPtr& joy);
    ros::Publisher joys_pub;
    ros::Subscriber joy_sub_;

  };
  
  
  
}
