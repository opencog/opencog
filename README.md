#opencog-to-minecraft#

A ROS-based module linking OpenCog and the Minecraft world. This code serves as a starting point for integrating the two environments. The code is in a 'heavy experimentation' phase, and will therefore change drastically over the next couple of months. The steps to set up and run the module are currently rather complicated, but this will change as the code stabilizes.

The modules in src/spockextra, mapnode.py and testbot3.py is made by [LucidBlue](https://github.com/LucidBlue/ros-to-minecraft/).

OpenCog Python bindings, changes to the SpaceServer, and perception manager created by
[chenesan](https://github.com/chenesan/opencog-to-minecraft).

And the netcat.py module is made by [Linas](https://github.com/opencog/ros-behavior-scripting/blob/master/face_track/netcat.py).

##prerequisite##

To start this, you should have installed:

####ROS http://www.ros.org/

After installing ROS you have to create a workspace and move this repository into the new workspace.

####Octomap http://wiki.ros.org/octomap

In ubuntu you can directly install Octomap by:

`sudo apt-get install ros-XXX-octomap

(XXX depends on your ROS distribution)

####Opencog https://github.com/opencog/opencog

For installing Opencog you also have to install 

####atomspace https://github.com/opencog/atomspace

and

####cogutils https://github.com/opencog/cogutils

To run the minecraft you should have a MineCraft server:

####Minecraft server(official) https://minecraft.net/download

You also need a python API to connect with Minecraft server, the Spock:

####Spock https://github.com/SpockBotMC/SpockBot

Besides the above package, I haven't pull all of my work in opencog side into the opencog master repository. So you have to add all the files in opencog/ manually.

Because for now(2015-08-07) the SpaceMap breaks the old embodiment, so it will fail to build the whole opencog. To make this Minecraft embodiment work, we only need to build these libs:

*cogserver
*spacetime
*spacetime-types
*SpaceMap
*spacetime-cython
*spatial-cython

So, after adding files under your opencog directory, build it by following procedure (assume the working directory is opencog/):

    mkdir build
    cmake ..
    cd build
    cd opencog/server
    make
    cd ../spatial
    make
    cd ../spacetime
    make
    cd ../cython/opencog
    make

##step to run##

1. start roscore and Minecraft Server

2. start cogserver, remember put the minecraft_bot/src/ in the PYTHON_EXTENSION_DIRS variable in the lib/opencog.conf so we can import those python file we need; Besides you have to execute cogserver under your opencog/build/ directory or the cogshell will fail to load the commands.

3. run 

   `echo -e 'py\n' | cat - opencog_initializer.py | netcat localhost 17001` 

(Thanks to Linas' netcat module!)to start the ROS node and intialize perception manager in cogserver.

4. Follow instructions in minecraft_bot to start ROS nodes and initialize Spock.

5. Connect with cogserver by:

   `rlwrap telnet localhost 17001`

6. For now(2015-08-07) you can use the py shell in cogserver to get visible block atoms manually by following python code:

    #you have to input the position and look of bot
    vis_block_srv_response = getVisBlocksFromSrv(x,y,z,yaw,pitch)
    pm.processMapMessage(vis_block_srv_response.visible_blocks)

7. Return to the cogshell, and use 
   'list -a
   You will see the visible block atoms and associated predicate nodes.

##TODO##

1. Use the idmap in minecraft_bot/src/embodiment-testing/ to replace the block id in perception manager. So we can have a useful atom name.

2. Pass every message to Opencog with the ROS/Minecraft timestamp to record the timestamp in atomspace/TimeServer correctly.

3. Make Opencog get visible block automatically. Not sure if it's better to use a MindAgent which updates block info in each cycle or just use a loop.

4. Make a [behavior tree](http://wiki.opencog.org/w/Behavior_tree) to test the pipeline from perception to action. 

5. add a void block handle in SpaceMap to distinguish the "unknown" block and the "air" block. (For example, if we only use Handle::UNDEFINED to express all non-void block, what does it mean when we get a undefined handle from spaceMap::get_block(position)? Does it mean (1) we've known there's no block in this position? or (2) we haven't record block in this position and we don't know if there's a block in the embodied environment?) To distinguish it, I guess it's better to use different block handle to distinguish void and unknown block.

6. write a script for starting all of the things(ROS nodes/Spock/cogserver/Minecraft server)

