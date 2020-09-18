# SVC 2020 Repository Set Up
This README includes everything you need to get your computer set up with ROS and the programs necessary to start messing around with the simulation. 

### Computer Setup
Download the correct Ubuntu distribution. All that really matters is that you download Ubuntu 18.04. The numbers after the "4" don't really matter. However, I chose to use [Ubuntu 18.04.5.](https://releases.ubuntu.com/18.04/) There are several methods you can use to run Ubuntu. You could use a virtual machine or do a native installation. I chose to install Ubuntu alongside Windows on my laptop, this is called "dual booting"
- fdgfdf




In order to generate the complete workspace, you must perform the follwoing tasks:

Step 1. Create a Workspace by ruunning the 'mkdir -p ~/your_ws_name/src' in the terminal

Step 2. Clone this repo into the src folder of you workspace

Step 3. Follow The link below to google drive and copy the files to the root of the workspace (Where src, devel, build folders are located): https://drive.google.com/drive/folders/1lVTvs0i6YZtJNHVeyR2fNH0mVQop1Tdr?usp=sharing

Step 4. catkin_make

Step 5. Source the devel folder in .bashrc, this step is for convenience and can be ignored if the user wishes to source the workspace in the terminal

Step 5. Read through the rest of this document for detailed steps

##################################################################

TO RUN ANY FILES, You must first Launch a scenario launch file in the gazebo world ex. 'roslaunch igvc_self_drive_gazebo f7_gazebo.launch'

##################################################################

Lane Detection
Status: Functional

TO RUN: roslaunch lane_detection test_projection.launch

#################################################################

Object Detection & Classification (YOLO):
Status: Detects Construction Barrels, Stop Signs

Does NOT Detect Pedstrians, will need to be addressed

TO RUN: roslaunch self_drive_launch yolo.launch // Launch rviz from the terminal -> click 'add' -> 'By Topic' -> /yolo ->image

#################################################################

Point Cloud from Cepton LIDAR
Status: Sensor is fucntional, but requires implementation of Euclidian Clustering

#################################################################

Xbox Controller (Joytest)

Status: File is outdated, need to update with John's code on other SVC laptop

#################################################################

catkin_make command is unsucsseful due to some error:
'CMake Error at /usr/local/lib/cmake/gazebo/gazebo-config.cmake:145 (message): Library 'gazebo' in package GAZEBO is not installed properly Call Stack (most recent call first): igvc_self_drive_sim/igvc_self_drive_gazebo_plugins/CMakeLists.txt:13 (find_package)

-- Configuring incomplete, errors occurred! See also "/home/svc/SVC_2020/build/CMakeFiles/CMakeOutput.log". See also "/home/svc/SVC_2020/build/CMakeFiles/CMakeError.log". Makefile:2546: recipe for target 'cmake_check_build_system' failed make: *** [cmake_check_build_system] Error 1'

##################################################################

All other packages in the src folder are not used at the moment


