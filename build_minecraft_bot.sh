 #!/bin/bash

mkdir catkin_ws
mkdir catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
catkin_create_pkg minecraft_bot std_msgs rospy roscpp
cp -r ../../minecraft_bot .
cd ..
catkin_make
