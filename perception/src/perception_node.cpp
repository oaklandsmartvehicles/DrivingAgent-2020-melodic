// ROS and node class header file
#include <ros/ros.h>
#include "Perception.h"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "perception");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  // Instantiate node class
  perception::Perception node(n, pn);
  
  // Spin and process callbacks
  ros::spin();
}
