#! /bin/bash
#
# eva.sh
#
# ROS + blender launch script for the Hanson Robotics Eva blender head.
# This shell script wiill start up the various bits and pieces needed to
# get things to work.
#
# It needs to run in the catkin_ws directory where the various ROS nodes
# and blender models were installed. It assumes that catkin_make was
# already run.
#
# If you have not yet installed the pre-requisites, then do this:
# cd ~ ; mkdir src ; cd src
# echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
# mkdir catkin ; cd catkin ; mkdir src ;
# cd src
# git clone git@github.com:hansonrobotics/blender_api
# git clone git@github.com:hansonrobotics/blender_api_msgs
# git clone git@github.com:hansonrobotics/pau2motors
# cd ..
# catkin build

# Change this for your setup!
export CATKINDIR="."
export BLENDIR="$CATKINDIR/../hr/blender_api"
export OCBHAVE="$CATKINDIR/../opencog/ros-behavior-scripting/"
export PYTHONPATH=$PYTHONPATH:`pwd`/$OCBHAVE/src

# Without this, some ros messages seem to run astray.
export ROS_IP=127.0.0.1

source devel/setup.sh
echo "Starting... this will take 15-20 seconds..."

# Use byobu so that the scroll bars actually work.
byobu new-session -d -n 'ros' 'roscore; $SHELL'
sleep 4;

# Single Video (body) camera and face tracker.
tmux new-window -n 'trk' 'roslaunch robots_config tracker-single-cam.launch; $SHELL'

# Publish the geometry messages. This includes tf2 which holds
# the face locations.
tmux new-window -n 'geo' 'roslaunch robots_config geometry.launch gui:=false; $SHELL'

### Start the blender GUI.
tmux new-window -n 'eva' 'cd $BLENDIR && blender -y Eva.blend -P autostart.py; $SHELL'

# Start the cogserver.
# It seems to take more than 5 seconds to load all scripts!?
cd $OCBHAVE/src
tmux new-window -n 'cog' 'guile -l btree-eva.scm; $SHELL'
sleep 10

# Run the various sensory-input modules, including the face-tracker.
# tmux new-window -n 'face' '$OCBHAVE/face_track/main.py; $SHELL'
tmux new-window -n 'sen' '../sensors/main.py; $SHELL'

# Telnet shell
tmux new-window -n 'tel' 'rlwrap telnet localhost 17020; $SHELL'

# Spare-usage shell
tmux new-window -n 'bash' '$SHELL'

# Fix the annoying byobu display.
echo "tmux_left=\"session\"" > $HOME/.byobu/status
echo "tmux_right=\"load_average disk_io\"" >> $HOME/.byobu/status
tmux attach

echo "Started"
