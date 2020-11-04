#include <ros/ros.h>
#include "joystick.hpp"

int main (int argc , char** argv)
{
  
  ros::init(argc, argv, "joystic__");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  Joystick::joystick node(n, pn);
  ros::spin();
 
  
}