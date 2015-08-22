#opencog-to-minecraft#

A ROS-based module linking OpenCog and the Minecraft world. This code serves as a starting point for integrating the two environments. The code is in a 'heavy experimentation' phase, and will therefore change drastically over the next couple of months. The steps to set up and run the module are currently rather complicated, but this will change as the code stabilizes.

For now this repository are mainly made and managed by [LucidBlue](https://github.com/LucidBlue) and [chenesan](https://github.com/chenesan). Later this repo may be included under the [Opencog](https://github.com/opencog).

##prerequisite##

To start this, you should have installed:

####ROS http://www.ros.org/

After installing ROS you have to create a workspace and move the repo into the new workspace.

####Octomap http://wiki.ros.org/octomap

In ubuntu you can directly install Octomap by:

`sudo apt-get install ros-XXX-octomap

(XXX depends on your ROS distribution e.g. indigo/fuerte)

####Opencog https://github.com/opencog/opencog

Before installing Opencog you have to install

####atomspace https://github.com/opencog/atomspace

and

####cogutils https://github.com/opencog/cogutils

To run the MineCraft server:

####Minecraft server(official) https://minecraft.net/download

A python API to connect with Minecraft server, Spock:

####Spock https://github.com/SpockBotMC/SpockBot

Besides the above package, There are some code in Opencog side has not been pulled into Opencog master repository. So you have to add all the files in opencog/ manually.

Because for now(2015-08-07) the new OctoMap breaks the old embodiment, so it will fail to build the whole opencog. To make this Minecraft embodiment work, we only need to build these libs:

*spacetime
*spacetime-types
*SpaceMap
*spacetime-cython
*spatial-cython

So, after adding files under your opencog directory, build it by following procedure (assume the working directory is opencog/):

    mkdir build
    cmake ..
    cd build
    cd ../spatial
    make
    cd ../spacetime
    make
    cd ../cython/opencog
    make

##step to run##

1. Start roscore and Minecraft Server.

2. Follow instructions in minecraft_bot to start ROS nodes and initialize Spock. Now you should see the bot appeared in your Minecraft. You can find the bot by move to the place bot spawned(showed in the Minecraft Server).

3. Start the opencog_initializer.py.

4. You should see the bot start to move toward 90 degree directions(you can see the information by pressing F3 in your Minecraft client.)

5. Put a "Gold" block in front of the bot.(0 degree direction)

6. You should see the bot stops and walks toward the block. The bot is attracted by the target gold block. Then the bot leaves the gold block and keep going since as time going the attention value of block decreases. For now (20150822) that's all behaviors of the bot.

##TODO##

* Document all the code.

* Add more actions in actions ROS node and Opencog: mining, placing block, inventing items and more and more and more.

* Add more percetions in middle ROS node and Opencog: Entity, items, chat message and more and more and more.

* More visibility: For testing now we confine the bot's FOV so it only can get what it sees in near distance(distance < 10). It's great if we can make it see further and more accurate.

* Use the existing [planner](https://github.com/opencog/opencog/blob/master/opencog/embodiment/Control/OperationalAvatarController/OCPlanner.h) and [pathfinding](https://github.com/opencog/opencog/blob/master/opencog/spatial/3DSpaceMap/Pathfinder3D.cc) module in the action generator. Cython binding seems a possible way.

* Use the idmap in minecraft_bot/src/embodiment-testing/ to replace the block id in perception manager. So we can have a useful atom name.

* Pass every message to Opencog with the ROS/Minecraft timestamp to record the timestamp in atomspace/TimeServer correctly.

* add a void block handle in SpaceMap to distinguish the "unknown" block and the "air" block. (For example, if we only use Handle::UNDEFINED to express all non-void block, what does it mean when we get a undefined handle from spaceMap::get_block(position)? Does it mean (1) we've known there's no block in this position? or (2) we haven't record block in this position and we don't know if there's a block in the embodied environment?) To distinguish it, I guess it's better to use different block handle to distinguish void and unknown block.

* write a script for starting all of the things(ROS nodes/Spock/Opencog/Minecraft server) in differenct terminal(using tmux/screen)

* There are more subtle TODOs in the codes...We should move them to the github issues.

