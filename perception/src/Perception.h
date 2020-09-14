// Include guard to prevent multiple declarations
#pragma once

// ROS header
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "PointMap.hpp"
#include "pcl_ros/point_cloud.h"
#include <tf/transform_listener.h>
#include <yolo/Detections.h>
#include <std_msgs/ColorRGBA.h>

enum eYOLOClassification
{
  yoloConstructionBarrel = 0,
  yoloOneWayLeft = 1,
  yoloOneWayRight = 2,
  yoloRoadClosed = 3,
  yoloStopSign = 4,
  yoloPedestrian = 5,
  yoloNoTurns = 6
};

// Namespace matches ROS package name
namespace perception {

  class Perception {
  public:
    Perception(ros::NodeHandle& n, ros::NodeHandle& pn);

  private:
    
    void LIDARCallback(int id, const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&);
    void LIDARCallback0(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&);

    void YOLOCallback(const yolo::Detections::ConstPtr&);

    std_msgs::ColorRGBA GetYOLOClassColor(eYOLOClassification object_class);
    void PublishObjectArrows(const yolo::Detections::ConstPtr& detections, const std::vector<double> distances);

    tf::TransformListener tf_listener;
    tf::StampedTransform transform;
    bool lidar_transform_found;

    ros::Publisher map_pub;
    std::vector< ros::Subscriber > lidar_subs;

    ros::Publisher objects_pub;

    ros::Subscriber yolo_sub;
    ros::Publisher yolo_detection_window_pub;
    ros::Publisher yolo_detection_arrow_pub;
    double camera_x, camera_y, camera_z;
    

    bool camera_transform_found;

    PointMap point_map;
    std::pair<double, double> mapsize;

    std::map<int, int> lidar_id_to_update_id;
  };

}