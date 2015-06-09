#! /bin/bash
#
# eva.sh
#
# ROS + blender launch script for the Hanson Robotics Eva blender head.
# This shell script is automatically started within the docker container.
# It needs to run in the catkin_ws directory where the various ROS nodes
# and blender models were installed. It assumes that catkin_make was
# already run.

source devel/setup.sh
echo "Starting... this will take 15-20 seconds..."

# Use byobu so that the scroll bars actually work
byobu new-session -d -n 'roscore' 'roscore; $SHELL'
sleep 4;

# Single Video (body) camera and face tracker
tmux new-window -n 'trk' 'roslaunch robots_config tracker-single-cam.launch; $SHELL'
# Publish the geometry messages
tmux new-window -n 'geo' 'roslaunch robots_config geometry.launch gui:=false; $SHELL'
tmux new-window -n 'bhave' 'rosrun eva_behavior main.py; $SHELL'
tmux new-window -n 'eva' 'cd /catkin_ws/src/blender_api && blender -y Eva.blend -P autostart.py; $SHELL'

# Spare-usage shell
tmux new-window -n 'bash' 'sleep 4; rostopic  pub --once /behavior_switch std_msgs/String btree_on; $SHELL'

# Fix the annoying byobu display
echo "tmux_left=\"session\"" > $HOME/.byobu/status
echo "tmux_right=\"load_average disk_io date time\"" >> $HOME/.byobu/status
tmux attach

echo "Started"
