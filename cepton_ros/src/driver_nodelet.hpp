#pragma once

#include <string>

#include <nodelet/nodelet.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <jsoncpp/json/json.h>
#include <sensor_msgs/PointCloud2.h>
#include <cepton_sdk_api.hpp>
#include <tf/tf.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "cepton_ros/SensorInformation.h"
#include "cepton_ros/common.hpp"
#include "cepton_ros/point.hpp"

namespace cepton_ros {

/// SDK nodelet.
/**
 * Publishes sensor information and points topics.
 */
class DriverNodelet : public nodelet::Nodelet {
 public:
  ~DriverNodelet();

  void on_image_points(cepton_sdk::SensorHandle sensor_handle,
                       std::size_t n_points,
                       const cepton_sdk::SensorImagePoint *const image_points);

 protected:
  void onInit() override;

 private:
  void publish_sensor_information(
      const cepton_sdk::SensorInformation &sensor_info);
  void publish_points(uint64_t serial_number);
  void parseTransformsFile(const std::string& transforms_path);

 private:
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle;

  bool combine_sensors = false;

  cepton_sdk::api::SensorErrorCallback error_callback;
  cepton_sdk::api::SensorImageFrameCallback image_frame_callback;

  ros::Timer timer;
  tf2_ros::StaticTransformBroadcaster tf_broadcaster;

  std::vector<cepton_sdk::SensorImagePoint> image_points;
  std::vector<cepton_sdk::util::SensorPoint> points;
  CeptonPointCloud point_cloud;
  
  // JSON parameters
  std::map<uint64_t, std::string> frame_ids;
  std::map<uint64_t, ros::Publisher> points_publishers;
  std::map<uint64_t, ros::Publisher> sensor_info_publishers;
};
}  // namespace cepton_ros
