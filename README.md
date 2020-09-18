# SVC 2020 Repository Set Up
This README includes everything you need to get your computer set up with ROS and the programs necessary to start messing around with the simulation. This README tutorial was generated with the help of Samer Labban, Ben Grudzien, and John Brooks. 

## Computer Setup
Download the correct Ubuntu distribution. All that really matters is that you download Ubuntu 18.04 because that works with the version of ROS we are using called ROS Melodic. The numbers after the "4" don't really matter. However, I chose to use [Ubuntu 18.04.5.](https://releases.ubuntu.com/18.04/) There are several methods you can use to run Ubuntu. You could use a virtual machine or do a native installation. I chose to install Ubuntu alongside Windows on my laptop, this is called "dual booting"
- Virtual Machines are often laggy when running ROS. However, they come with less risk of making a mistake. [VM tutorial here.](https://www.youtube.com/watch?v=QbmRXJJKsvs)
- Native installations are reccomended. Here is a [dual boot tutorial.](https://www.youtube.com/watch?v=u5QyjHIYwTQ)

**Note:** Follow/refer to [this tutorial](https://www.linuxtechi.com/ubuntu-18-04-lts-desktop-installation-guide-screenshots/) for installation help. Remember to click the "Install third party software" checkbox. (This will save you a lot of time having to debugg stuff like GPU or camera drivers)

## Software Setup
This section covers how to get Ubuntu up and running with everything you need to run the ROS simulation in this repository. (Don't include the quotations in the terminal commands)

- **Step 1, Update Ubuntu**: Run 'sudo apt-get update' then 'sudo apt-get upgrade' 

Like 99% of this we don't need, but we're gonna take a sledgehammer to a nail because it's the easy way. The package manager is basically updating everything to the latest version. Now's a good time for a coffee or beer break depending on the time of day. When this is done, restart your computer.

- **Step 2, Install ROS**: Run 'bash <(wget -O - "http://secs.oakland.edu/~mtradovn/ece_6460/software_setup.bash")'

This is the ROS install script for Professor Radovnikovich's ROS class. This includes ROS melodic, some packages, and Visual Studio. If you already had a version of ROS installed, open your .bashrc file and remove the extra source /opt/ros/melodic/setup.bash line at the bottom that was added there by the setup script. That will override your existing ROS workspace.

- **Step 3, Install CUDA**: Run 'sudo add-apt-repository ppa:graphics-drivers/ppa', 'sudo apt update', 'sudo ubuntu-drivers autoinstall', 'sudo reboot', 'sudo apt install nvidia-cuda-toolkit gcc-6', 'nvcc --version'

You should now see some output and a version number like release 9.1, V9.1.85. Cool.

- **Step 4, Install Dependencies**: Run 'sudo apt-get install build-essential', 'sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev', 'sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev', 'auso'

These should already be installed, but run the commands anyways.

- **Step 5, Install YOLO Dependencies**: 'sudo apt-get install ros-melodic-costmap-converter'

if yolo doesn't build correctly then check that in the catkin_ws/src/yolo/CMakeLists.txt the path is correctly pointing to the yolo shared object.

    #change this to your darknet path!!!
    target_link_libraries(YOLONode
   	 ~/catkin_ws/src/yolo/darknet/libdarknet.so
    )

WE already have the Yolo files set up, but if you want to download stuff yourself, read below:

Yolo also requires a configuration and a weights file. The pre-trained configuration and weights files can be found here…
https://pjreddie.com/darknet/yolo/ If you download one of these sets of files. They must match. I recommend yolov3-tiny for crappy GPUs and yolov3-608 for high performance GPUs. In competition we will have our own cfg and weights files. At the time of writing those do not yet exist. Rename those files to run.cfg and run.weights. Put them in a new folder  ~/catkin_ws/darknet/


 


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


