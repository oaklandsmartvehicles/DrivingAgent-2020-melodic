cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_BUILD_TYPE Debug)

project(yolo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  xmlrpcpp
  message_generation
)

add_message_files(
  FILES 
  Detection.msg
  Detections.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

catkin_package(
	CATKIN_DEPENDS message_runtime
)

include_directories(
	${catkin_INCLUDE_DIRS}
)

#Compile a node written in C++
add_executable(YOLONode
	src/YOLONode.cpp
	src/yolo_node.cpp
	src/YOLO.cpp
)

add_dependencies(YOLONode ${PROJECT_NAME}_gencpp)

target_link_libraries(YOLONode
	${catkin_LIBRARIES}
	m
    pthread
    stdc++
	cudart
    cuda    
    cublas
	curand
)

target_link_libraries(YOLONode
	${CMAKE_CURRENT_SOURCE_DIR}/darknet/libdarknet.so
)


