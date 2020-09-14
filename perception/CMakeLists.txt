cmake_minimum_required(VERSION 2.8.3)
project(perception)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosmsg
  pcl_ros
  yolo
  nodelet
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Object.msg
  Objects.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/perception_nodelet.cpp
  src/Point.cpp
  src/PointMap.cpp
  src/PointMapCell.cpp
  src/Perception.cpp
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node
  src/perception_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)
