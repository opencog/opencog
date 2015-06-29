
The catkin (ROS) package for running the Minecraft bot

##Steps to run##

(assuming server, OpenCog, ROS, and SpockBot have been installed as detailed in the main directory)

1. Create a catkin package in your workspace (minecraft_bot) and include all the files from this directory

2. Run catkin_make

3. Run mapnode.py to start the ROS node that handles sending map data to the space server.

4. Run any of the testbot*.py files to initialize a Spock bot and test related plugins.

All source for our custom SpockBot plugins are in opencog-to-minecraft/spock/plugins/. ROS nodes are all in this directory.
