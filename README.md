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

####Opencog https://github.com/opencog/opencog

For installing Opencog you also have to install 

####atomspace https://github.com/opencog/atomspace

and

####cogutils https://github.com/opencog/cogutils

To run the minecraft you should have a MineCraft server:

####Minecraft server(official) https://minecraft.net/download

You also need a python API to connect with Minecraft server, the Spock:

####Spock https://github.com/SpockBotMC/SpockBot

Besides the above package, I haven't pull all of my work in opencog side into the opencog master repository. 
So you have to add these files in src/opencog/ as the following procedure:

1.put the classserver.pxd in atomspace/opencog/cython/opencog. It's for updating the atom types after adding spacetime module.

2.build the atomspace

3.put the CMakeList,spacetime.pyx/pxd, spatial.pyx/pxd, in opencog/opencog/cython/opencog. These are the cython binding of Space/Time Server and SpaceMap.

4.put the Octree.h and Octree.cc in opencog/opencog/spatial/3DSpaceMap. It's a small fix for a bug in Octree.

5.build the opencog

##step to run##

1. start roscore and Minecraft Server

2. start cogserver, remember put the src/ path in the PYTHON_EXTENSION_DIRS variable in the opencog.conf so we can import those python file we need; Besides you have to execute cogserver under your opencog/build/ directory or the cogshell will fail to load the commands.

3. run 

   `echo -e 'py\n' | cat - opencog_intializer.py | netcat localhost 17001` 

(Thanks to Linas' netcat module!)to start the ROS node and intialize perception manager in cogserver.

4. Follow instructions in minecraft_bot to start ROS and initialize Spock.

5. Then you can connect with cogserver by:

   `rlwrap telnet localhost 17001`

Now you should see the block atoms in the cogserver. You can also check them by using pyshell/guile shell.


