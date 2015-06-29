#opencog-to-minecraft#

Now this is just a temporary workflow to test the pipeline from Minecraft to Opencog. The integration has not finished. After the integration is stable we will move these codes into another repositories for future works.

The modules in src/spockextra, mapnode.py and testbot3.py is made by [LucidBlue](https://github.com/LucidBlue/ros-to-minecraft/).
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

4. start mapnode.py. It will start a ROS node to receive primary blocks messages, save them and send block information to opencog. So far we just send the whole world to opencog. And in the future we will add more information(Entity/players...) and visibility calculation in the ROS node.

5. start testbot3.py. It will start Spock and a new ROS node. This node will only be responsible for receiving Minecraft packets and sending primary blocks message to the mapnode.

6. Then you can connect with cogserver by:

   `rlwrap telnet localhost 17001`

Now you should see the block atoms in the cogserver. You can also check them by using pyshell/guile shell.


