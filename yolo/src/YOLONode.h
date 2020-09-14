// Include guard to prevent multiple declarations
#pragma once

// ROS header
#include <ros/ros.h>
#include <string>
#include <sensor_msgs/Image.h>
//#include <tf2_ros/transform_listener.h>

class YOLO;
// Namespace matches ROS package name
namespace yolo {

  class YOLONode {

  public:
    YOLONode(ros::NodeHandle& n, ros::NodeHandle& pn);
    ~YOLONode();

    std::string mLastError;

  private:
	// Node-specific stuff here
        bool InitializeYOLO();
		void ImageTopicCallback(const sensor_msgs::ImageConstPtr& msg);
        YOLO* mYOLO;

		ros::Publisher status_publisher;
		ros::Subscriber image_subscriber;
        ros::Publisher detections_publisher;
        ros::Publisher detected_image_publisher;
        //tf2_ros::TransformListener tf_listener;
        //tf2_ros::Buffer tf_buffer;
        
        bool mError;
  };



}

