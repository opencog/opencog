
The catkin (ROS) package for running the Minecraft bot

##Installation##

(assuming server, OpenCog, ROS, and SpockBot have been installed as detailed in the main directory)

1. Create a catkin package in your workspace (minecraft_bot) and include all the files from this directory:
    
    cd /your_catkin_ws_dir/src/
    catkin_create_pkg minecraft_bot std_msgs rospy roscpp
    cp -r /your_opencog-to-minecraft_dir/minecraft_bot . 
    cd /your_catkin_ws_dir
    catkin_make

   See the [ROS official tutorial](http://wiki.ros.org/catkin/Tutorials/CreatingPackage)

##Steps to run##

Before running these steps you should have run the following command to set up ROS environment:

    roscore &
    cd /your_catkin_ws_dir/
    source ./devel/setup.bash

1. Start actionsnode.py. It will start a ROS node to publish actions message to Spock and provide ROS services for Opencog. Opencog can use the services to do action.

2. Start visnode.py. It will start a ROS node to calculate what's visible for the bot and publish ROS message for Opencog.

3. Start mapnode.py. It will start a ROS node to receive primary blocks messages, save them and provide block information service for visnode.py.

4. Start test_mc_bot.py files to initialize the Spock bot and test custom plugins. It will start Spock and a ROS node (SpockControl.py). This node will only be responsible for receiving Minecraft packets and sending raw environment data to the map node.

All source for our custom SpockBot plugins are in spockextras/plugins/helpers. ROS nodes are all in this directory.



