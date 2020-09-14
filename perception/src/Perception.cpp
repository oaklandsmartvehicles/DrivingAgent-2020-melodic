// Header file for the class
#include "Perception.h"
#include <string>
#include "visualization_msgs/Marker.h"
#include "tf/tf.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <math.h>
#include <perception/Objects.h>

#define POINTER_LENGTH 1

// Namespace matches ROS package name
namespace perception {

  // Constructor with global and private node handle arguments
  Perception::Perception(ros::NodeHandle& n, ros::NodeHandle& pn)
  {
      lidar_transform_found = false;
      // ros::Subscriber lidar1 = n.subscribe("cepton/LIDAR/FrontCenter", 1, &Perception::LIDARCallback0, this);
      ros::Subscriber lidar1 = n.subscribe("/cepton/points_raw", 1, &Perception::LIDARCallback0, this);

      lidar_subs.push_back(lidar1);

      map_pub = n.advertise<visualization_msgs::MarkerArray>("perception/map", 1);

      objects_pub = n.advertise<perception::Objects>("perception/Objects", 1);

      camera_transform_found = false;
      yolo_sub = n.subscribe("yolo/detections", 1, &Perception::YOLOCallback, this);
      yolo_detection_window_pub = n.advertise<visualization_msgs::MarkerArray>("perception/object_windows", 1);
      yolo_detection_arrow_pub = n.advertise<visualization_msgs::MarkerArray>("perception/object_pointers", 1);

      point_map.SetHeightCutOff(2);
      mapsize = point_map.GetSize();
  }

  void Perception::YOLOCallback(const yolo::Detections::ConstPtr& detections)
  {
    /*if(!camera_transform_found)
    {
      try{
        tf::StampedTransform camera_transform;
        tf_listener.lookupTransform("base_footprint", "camera_front",  
          ros::Time(0), camera_transform);
        camera_transform_found = true;
        camera_x = camera_transform.getOrigin().getX();
        camera_y = camera_transform.getOrigin().getY();
        camera_z = camera_transform.getOrigin().getZ();
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
      }
    }*/
    std::vector<double> distances;
    point_map.UpdateCellDimensions();
    for( auto detection : detections->detections)
    {
      double angle_h = (detection.angle_left + detection.angle_right) / 2;
      //printf("Angle: %0.3f\n", angle_h);
    
      double distance = 100;
      for( auto cell : point_map.cell_map)
      {
        const int& cell_x = cell.first.first;
        const int& cell_y = cell.first.second;
        std::pair<double, double> angular_window = cell.second.GetAngularWindow();
        //printf("Window L: %0.3f\t Window R: %0.3f\n", cell.second.a2, cell.second.a1 );
        if( angular_window.first > angle_h || angular_window.second < angle_h)
          continue;

        if( cell.second.meta_data.max_z - cell.second.meta_data.min_z < 0.5)
          continue;

		    //printf("cell intersected");

        if( cell.second.GetDistance() < distance)
          distance = cell.second.GetDistance() - 0.5;
      }
      distances.push_back(distance);
    }


    PublishObjectArrows(detections, distances);
  }

  void Perception::LIDARCallback(int id, const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& point_cloud)
  {
    point_map.ClearMap();
    if(!lidar_transform_found)
    {
      // "cepton_front_center"
      try{
        tf_listener.lookupTransform("base_footprint", point_cloud->header.frame_id,  
          ros::Time(0), transform);
        lidar_transform_found = true;
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
      }
    }


    if(lidar_id_to_update_id.find(id) != lidar_id_to_update_id.end())
      point_map.RemoveUpdate(lidar_id_to_update_id[id]);

    std::vector<Point> map_points;
    /*tf::Vector3 origin = transform.getOrigin();
    tf::Quaternion rotation = transform.getRotation();*/
    tf::Vector3 transformed;
    
    for( auto point : point_cloud->points )
    {
        
        transformed.setX(point.x);
        transformed.setY(point.y);
        transformed.setZ(point.z);
        /*
        transformed = tf::quatRotate(transform.getRotation(), transformed);
        transformed.x -= transform.getOrigin().x;
        transformed.y -= transform.getOrigin().y;
        transformed.z -= transform.getOrigin().z;*/

        transformed = transform * transformed;
        Point map_point( transformed.getX(), transformed.getY(), transformed.getZ() );
        map_points.push_back(map_point);
    }

    int update_id = point_map.AddLIDARPoints(map_points);
    lidar_id_to_update_id[id] = update_id;

    
    visualization_msgs::MarkerArray cubes;
    std::pair<double, double> xy_size = point_map.GetCellSize();
    int i = 0;
    double x_offset = mapsize.first / 2;
    double y_offset = mapsize.second / 2;

    visualization_msgs::Marker cube;
    cube.action = visualization_msgs::Marker::DELETEALL;
    cubes.markers.push_back(cube);

    for( auto cell : point_map.cell_map )
    {
        visualization_msgs::Marker cube;
        cube.ns = "PointMap";
        cube.pose.position.x = (cell.first.first * xy_size.first) - x_offset;
        cube.pose.position.y = (cell.first.second * xy_size.first)  - y_offset;
        cube.pose.position.z = cell.second.meta_data.max_z;
        cube.id = i++;
        cube.scale.x = xy_size.first;
        cube.scale.y = xy_size.second;
        cube.scale.z = 1;
        cube.header.frame_id = "base_footprint";
        cube.header.stamp = ros::Time();
        cube.type = visualization_msgs::Marker::CUBE;
        cube.action = visualization_msgs::Marker::ADD;
        cube.color.a = 0.3;
        if(cell.second.meta_data.max_z - cell.second.meta_data.min_z > 0.5)
        {
          cube.color.a = .5;
          cube.color.r = 1;
          cube.color.g = 0;
          cube.color.b = 0;
        }
        else
        {
          cube.color.a = .5;
          cube.color.r = 0;
          cube.color.g = 1;
          cube.color.b = 0;
        }
        cube.scale.z = cell.second.meta_data.max_z - cell.second.meta_data.min_z;
        cube.pose.position.z = cell.second.meta_data.min_z + (cube.scale.z / 2);  
      
        cube.color.b = 0;
      
        cubes.markers.push_back(cube);
    }
    map_pub.publish(cubes);
  }
  

  void Perception::LIDARCallback0(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& point_cloud)
  {
      LIDARCallback(0, point_cloud);
  }
  
  
  std_msgs::ColorRGBA Perception::GetYOLOClassColor(eYOLOClassification object_class)
  {
    std_msgs::ColorRGBA color;
    color.a = 1;
    switch(object_class)
    {
      case yoloConstructionBarrel:
        color.r = 1;
        color.g = 0.5;
        color.b = 0;
        break;
      case yoloOneWayLeft:
        color.r = 1;
        color.g = 0;
        color.b = .5;
        break;
      case yoloOneWayRight:
        color.r = 0;
        color.g = 1;
        color.b = 1;
        break;
      case yoloRoadClosed:
        color.r = 0;
        color.g = 1;
        color.b = 0;
        break;
      case yoloStopSign:
        color.r = 1;
        color.g = 0;
        color.b = 0;
        break;
      case yoloPedestrian:
        color.r = 0.5;
        color.g = 0;
        color.b = 1;
        break;
      case yoloNoTurns:
        color.r = 1;
        color.g = 1;
        color.b = 1;
        break;
    }
    return color;
  }

  void Perception::PublishObjectArrows(const yolo::Detections::ConstPtr& detections, const std::vector<double> distances)
  {
    visualization_msgs::MarkerArray arrows;
    visualization_msgs::Marker delete_all;
    delete_all.action = visualization_msgs::Marker::DELETEALL;
    arrows.markers.push_back(delete_all);

    geometry_msgs::Point point1, point2;
    point1.x = 0;
    point1.y = 0;
    point1.z = 0;
    int i = 0;
    perception::Objects objects_update;
    objects_update.header.stamp = ros::Time();
    for( auto detection : detections->detections)
    {
      eYOLOClassification object_classification = (eYOLOClassification)detection.class_id;

      visualization_msgs::Marker pointer;
      pointer.type = visualization_msgs::Marker::LINE_LIST;
      pointer.points.push_back(point1);

      double angle_h = (detection.angle_left + detection.angle_right) / 2;
      double angle_v = (detection.angle_top + detection.angle_bottom) / 2;

      point2.x = point1.x + (cos(angle_h) * (distances[i]-.5));
      point2.y = point1.y + (sin(angle_h) * (distances[i]-.5));
      point2.z = point1.z + (sin(angle_v) * (distances[i]-.5));

      pointer.points.push_back(point2);

      pointer.id = i++;
      pointer.scale.x = 0.05;

      pointer.colors.push_back(GetYOLOClassColor(object_classification));
      pointer.colors.push_back(GetYOLOClassColor(object_classification));
      
      pointer.action = visualization_msgs::Marker::ADD;

      pointer.ns = "object_arrows";
      pointer.header.frame_id = "camera_front";
      pointer.header.stamp = ros::Time();

      printf("Object %i \t Distance: %0.3f\n", detection.class_id, distances[i-1]);

      perception::Object object;
      object.class_id = detection.class_id;
      object.confidence = detection.confidence;
      object.x = point2.x;
      object.y = point2.y;
      objects_update.objects.push_back(object);

      arrows.markers.push_back(pointer);
    }
    objects_pub.publish(objects_update);
    yolo_detection_arrow_pub.publish(arrows);
  }

}//end of namespace
