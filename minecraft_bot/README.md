
The catkin (ROS) package for running the Minecraft bot

##Installation##

(assuming server, OpenCog, ROS, and SpockBot have been installed as detailed in the main directory)

Run the build_minecraft_bot.sh script in the root folder of this repo.

##Steps to run##

First you will need to start roscore with the following command:

    roscore

Then it is recommended to start each node in it's own terminal.

    source ./catkin_ws/devel/setup.bash
    rosrun minecraft_bot NODENAME.py

1. Start actionsnode.py. It will start a ROS node to publish actions message to Spock and provide ROS services for Opencog. Opencog can use the services to do action.

2. Start visnode.py. It will start a ROS node to calculate what's visible for the bot and publish ROS message for Opencog.

3. Start mapnode.py. It will start a ROS node to receive primary blocks messages, save them and provide block information service for visnode.py.

4. Start test_mc_bot.py files to initialize the Spock bot and test custom plugins. It will start Spock and a ROS node (SpockControl.py). This node will only be responsible for receiving Minecraft packets and sending raw environment data to the map node.

All source for our custom SpockBot plugins are in spockextras/plugins/helpers. ROS nodes are all in this directory.



