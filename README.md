In order to replicate to generate the complete workspace, you must perform the follwoing tasks:

Step 1. Create a Workspace using 'mkdir -p ~/dummy_name/src'
Step 2. Clone this repo into the src folder of you workspace
Step 3. Follow 'This' link to google drive and copy the files to the root of the workspace (Where src, devel, build folders are located)
Step 4. catkin_make 
Step 5. Read through the rest of this document for detailed steps

## TO RUN ANY FILES, You must first Launch a Scenrio launch file in the gazebo world ex. roslaunch igvc_self_drive_gazebo f7_gazebo.launch
 

## Lane Detection

Status: Functional

TO RUN: roslaunch lane_detection test_projection.launch 

#################################################################

## Object Detection & Classification (YOLO):

Status: Detects Construction Barrels, Stop Signs

Does NOT Detect Pedstrians, will need to be addressed

TO RUN: roslaunch self_drive_launch yolo.launch // Launch rviz from the terminal -> click 'add' -> 'By Topic' -> /yolo ->image 

#################################################################

## Point Cloud from Cepton LIDAR

Status: Fucntional, but requires Euclidian Clustering to be useful and implement sensor fusion

#################################################################

## catkin_make command is unsucsseful due to some error:

'CMake Error at /usr/local/lib/cmake/gazebo/gazebo-config.cmake:145 (message):
  Library 'gazebo' in package GAZEBO is not installed properly
Call Stack (most recent call first):
  igvc_self_drive_sim/igvc_self_drive_gazebo_plugins/CMakeLists.txt:13 (find_package)


-- Configuring incomplete, errors occurred!
See also "/home/svc/SVC_2020/build/CMakeFiles/CMakeOutput.log".
See also "/home/svc/SVC_2020/build/CMakeFiles/CMakeError.log".
Makefile:2546: recipe for target 'cmake_check_build_system' failed
make: *** [cmake_check_build_system] Error 1'

##################################################################

## All other packages in the src folder are not used at the moment



