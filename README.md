The work is based on the https://github.com/LucidBlue/ros-to-minecraft/
And the changes are:

1. add ROS plugin in spock.
   The ROS plugin is for putting all the ROS initialization code in a place. So once we want to add a new publisher/subscriber, we just need to add Publisher and Subscriber settings in OCPublishers/OCSubscribers.py.

2. add Visibility plugin in spock.
   The Visibility plugin is for calculating what blcoks player see.
   It will update the visibility messages in a stable rate(the default setting is 0.1sec)

3. add visibility message in msg/

4. add simple perception code in opencog side.
   Once the opencog (cog_perception.py) receive the visibility messages, it will check if there's the target (Default is sand block) in its vision.
   If the bot can see the target, it will send an action message to minecraft.

With these changes, we can prove that the opencog can perceive visual messages from minecraft and act according to the content of messages.

To demo this, follow the instruction(Assume you've do the instruction in the https://github.com/LucidBlue/ros-to-minecraft/):

1. Put the files in src/spock_plugin/helpers into the spock/plugin/helpers folder
2. Change the spock/plugin/core/event.py to the original version in SpockBot repo, since the ROS initialization code has been moved to ROS plugin.
3. Execute catkin_make, start roscore and start minecraft server.
4. Start cog_perception.py and spock_listener.py
5. Start your minecraft game and login to your server
6. Now you can try to put the target(sand block) in the place the bot can see it. If the bot fins it, it should start to shake its hand. 


