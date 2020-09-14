cmake_minimum_required(VERSION 2.8.3)
project(gps_kalman_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  dynamic_reconfigure
  message_filters
)

generate_dynamic_reconfigure_options(
  cfg/Ekf.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/gps_kf_node.cpp
  src/GpsKalmanFilter.cpp
)
add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(
  TARGETS
    ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

