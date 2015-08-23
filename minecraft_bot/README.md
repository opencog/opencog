
The catkin (ROS) package for running the Minecraft bot

##Steps to run##

(assuming server, OpenCog, ROS, and SpockBot have been installed as detailed in the main directory)

1. Create a catkin package in your workspace (minecraft_bot) and include all the files from this directory

2. Run catkin_make

3. Start embodiment-testing/actionsnode.py. It will start a ROS node to publish actions message to Spock and provide ROS services for Opencog. Opencog can use the services to do action.

4. Start embodiment-testing/visnode.py. It will start a ROS node to calculate what's visible for the bot and publish ROS message for Opencog.

5. Start embodiment-testing/mapnode.py. It will start a ROS node to receive primary blocks messages, save them and provide block information service for visnode.py.

6. Start any of the testbot.py files to initialize the Spock bot and test custom plugins. It will start Spock and a ROS node (SpockControl.py). This node will only be responsible for receiving Minecraft packets and sending raw environment data to the map node.

All source for our custom SpockBot plugins are in spockextras/plugins/helpers. ROS nodes are all in this directory.



