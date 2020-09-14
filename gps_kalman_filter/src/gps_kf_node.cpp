#include <ros/ros.h>
#include "GpsKalmanFilter.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gps_kalman_filter");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  gps_kalman_filter::GpsKalmanFilter node(n, pn);
  
  ros::spin();
}