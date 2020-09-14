#/bin/bash
source /opt/ros/melodic/setup.bash
cd home/ros
catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release
catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --make-args tests
catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --make-args run_tests -j1
