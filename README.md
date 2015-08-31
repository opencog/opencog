#opencog-to-minecraft#

A ROS-based module linking OpenCog and the Minecraft world. This code serves as a starting point for integrating the two environments. The code is in a 'heavy experimentation' phase, and will therefore change drastically over the next couple of months. The steps to set up and run the module are currently rather complicated, but this will change as the code stabilizes.

For now this repository are mainly made and managed by [LucidBlue](https://github.com/LucidBlue) and [chenesan](https://github.com/chenesan). Later this repo may be included under the [Opencog](https://github.com/opencog).

##Prerequisite##

####Ubuntu http://www.ubuntu.com

It's recommended to set up environment under [Ubuntu Trusty](http://releases.ubuntu.com/14.04/).

You can also use the [Opencog docker container](http://wiki.opencog.org/w/Building_OpenCog#Docker_.2864_bit_systems_only.29), which containing Ubuntu Trusty and Opencog. Note that when starting a docker container, you have to publish port 25565 to make the Minecraft Server in the container use the port 25565 in your machine:

`docker create --name opencog -p 25565:25565-p 17001:17001 -p 5000:5000 -it $USER/opencog`

####ROS http://www.ros.org/

Now(20150823) ROS indigo distro is OK and recommended for Minecraft embodiment. Not sure if other distro will work or not.

Install instruction of ROS indigo is [here](http://wiki.ros.org/indigo/Installation/Ubuntu).

After installing ROS you have to create a workspace ([tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)) and install the minecraft_bot package by the instruction in [official tutorial](http://wiki.ros.org/catkin/Tutorials/CreatingPackage) and [minecraft_bot/README](https://github.com/OC2MC/opencog-to-minecraft/tree/master/minecraft_bot).

####Octomap http://wiki.ros.org/octomap

In ubuntu you can directly install Octomap by:

`sudo apt-get install ros-indigo-octomap

(Assume you have setuped the sources.list when you install ROS, or you will fail to find the package.)

If you are not using ROS indigo distro, just change the `ros-indigo-octomap` to `ros-your_distro-octomap`

####Opencog https://github.com/opencog/opencog

There are some Octomap codes in Opencog side has not been pulled into Opencog official repository. So you have to add all of files under [opencog-to-minecraft/opencog](https://github.com/OC2MC/opencog-to-minecraft/tree/master/opencog)) in the same place in your Opencog code base manually.

Because for now(2015-08-23) the new OctoMap breaks the old embodiment, so it will FAIL to build the whole Opencog. To make this Minecraft embodiment work, we only need to partially build Opencog Space/TimeServer, spacetime-types, SpaceMap and cython binding,

So, after adding files under your opencog directory, build it by following procedure (assume the working directory is under your Opencog):

    mkdir build
    cmake ..
    cd build
    cd ../spatial
    make
    cd ../spacetime
    make
    cd ../cython/opencog
    make


Before installing Opencog you have to install [atomspace](https://github.com/opencog/atomspace) and [cogutils](https://github.com/opencog/cogutils). Just install them by the instruction in README.

####Minecraft server(official) https://minecraft.net/download

Install instuction of Minecraft server is [here](http://minecraft.gamepedia.com/Tutorials/Setting_up_a_server)

You can also choose other Minecraft server manager you prefer.

####Spock https://github.com/SpockBotMC/SpockBot

A python API to connect with Minecraft server.

##Step to run##

It's recommended to use `tmux or `screen to run multiple ROS nodes in multiple terminal if you use Opencog docker container.

1. add the following PYTHONPATH:

    `/usr/local/share/opencog/python`

    `/your_opencog/build/opencog/cython/`

    `/your_opencog-to-minecraft/`

   and setup the ROS environment:

    `roscore &`
    `source /where_you_create_catkin_ws/devel/setup.bash`

   and set up your path to action schema, in opencog_python_eval.conf:
   
   `PYTHON_PRELOAD_FUNCTIONS = /path/to/your/minecraft_obt/src/action_schemas.py`

2. Start Minecraft Server.

3. Follow instructions in [minecraft_bot](https://github.com/OC2MC/opencog-to-minecraft/tree/master/minecraft_bot) to start ROS nodes and initialize Spock. Now you should see the bot appeared in your Minecraft. You can find the bot by move to the place bot spawned(showed in the Minecraft Server).

4. Start the opencog_initializer.py.

5. You should see the bot start to move toward 90 degree directions(you can see the information by pressing F3 in your Minecraft client.)

6. Put a "Gold" block in front of the bot.

7. You should see the bot stops and walks toward the block. The bot is attracted by the target gold block. Then the bot leaves the gold block and keep going since as time going the attention value of block decreases. For now (20150822) that's all behaviors of the bot.

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

