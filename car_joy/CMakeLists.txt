cmake_minimum_required(VERSION 2.8.3)
project(car_joy)

### [Mandatory] List all ROS packages this package is dependent on.
#               This list should match the packages with <depend> tags in package.xml
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
)

### [Recommended] Use C++ 11 compiler
add_compile_options(-std=c++11)

### [Mandatory] Identify this CMake project as a catkin package
catkin_package()

#### [Mandatory] Include exported include directories from other packages
#                this package is dependent on.
include_directories(
  ${catkin_INCLUDE_DIRS}
)

### Instruct catkin_make to compile a node executable program
#   First argument is the name of the resulting executable
#    After the executable name, list all .cpp files required
#        to compile the node. Keep in mind that the list of 
#        source files must consist of relative paths to the
#        files from the root of the package folder
#add_executable(example_node
 # src/example_node.cpp
 # src/example.cpp
#)
#add_dependencies(example_node
 # ${catkin_EXPORTED_TARGETS}
#)
#target_link_libraries(example_node
 # ${catkin_LIBRARIES}
#)

add_executable(joystick_
  src/joystick_.cpp
  src/joystick.cpp
)
add_dependencies(joystick_
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(joystick_
  ${catkin_LIBRARIES}
)
