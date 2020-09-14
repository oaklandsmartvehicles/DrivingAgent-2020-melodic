// ROS and node class header file
#include <ros/ros.h>
#include "YOLONode.h"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "yolo");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  // Instantiate node class
  yolo::YOLONode node(n, pn); 
  
  // Spin and process callbacks
  ros::spin();
  ros::waitForShutdown();
}
