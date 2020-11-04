cmake_minimum_required(VERSION 2.8.3)
project(drivebywire)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  rospy
)

generate_messages(  
    DEPENDENCIES
    std_msgs
)

catkin_package()

