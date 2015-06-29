
The catkin (ROS) package for running the Minecraft bot

##Steps to run##

(assuming server, OpenCog, ROS, and SpockBot have been installed as detailed in the main directory)

1. Create a catkin package in your workspace (minecraft_bot) and include all the files from this directory

2. Run catkin_make

3. Start mapnode.py. It will start a ROS node to receive primary blocks messages, save them and send block information to opencog. So far we just send the whole world to opencog. And in the future we will add more information(Entity/players...) and visibility calculation in the ROS node.

4. Start any of the testbot.py files to initialize the Spock bot and test custom plugins. It will start Spock and a ROS node (SpockControl.py). This node will only be responsible for receiving Minecraft packets and sending raw environment data to the map node.


All source for our custom SpockBot plugins are in opencog-to-minecraft/spock/plugins/. ROS nodes are all in this directory.



