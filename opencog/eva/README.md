Performance Scripting Interface
===============================
Experimental user interface for creating performance scripts to control
the Hanson Robotics Eva robot head. See:

* https://github.com/hansonrobotics/blender_api
* https://github.com/opencog/docker/tree/master/indigo/eva

Goal: Create a Web UI that will allow theatrical performance artists
to script theatrical performances for the Eva head.  An example of a
simple theatrical performance might be ''When a someone enters the room,
briefly glance thier way, give a faint smile, then resume with the
previous activity.''

The output of the script should be sequences of ROS messages that
control the Eva head (turn, look, smile), as well as the timing, pitch
and modulation of speech.

Creating the above might be a relatively straight-forward prorammming
task, were it not for one additional requirement: the scripts should be
executable within the OpenCog Atomspace framework. This is because we
want, as a long-range goal, for OpenCog to be able to control the robot
behavior itself, starting with more basic Psi-Theory urges and desires,
rather than having Eva be an animated puppet.

This means that the authoring tool needs to be able to work with not
just some convenient scripting language, but it should work very
explicitly with a certain OpenCog-based scripting language.

Status
------
This is version 0.0.1 of the project. Almost nothing is here yet.
There's going to be lots of miscellaneous experiments and a
mish-mash of code.

Running
-------
* Use the scripts/eva.sh file, after adjusting paths in it for your
  installation. Or use the below:

* Start the webcam, pi_vision, and tf2 tracking nodes as usual.
* Start the Eva blender node, as usual.
* Start the cogserver.
* Load the scripts shown in scripts/eva.sh
* Start the opencog face-tracking node in the `face_track` directory.

Owyl implementation
-------------------
The previous implementation used Owyl trees to represent behaviors.
There was no GUI or API for this: the behaviors are hard-coded in
python.  It can be found here:

https://github.com/hansonrobotics/eva_behavior
Here's a summary of the behaviors it scripts:

 * If the scene was empty and someone arrives, Eva interacts with the
   new arrival. Start by showing 'surprised' expression.

 * If Eva is currently interacting with someone, and someone else
   arrives, she will either (dice-roll): glance at the newcomer or
   ignore them.   The probability of a glance increases if she has
   been interacting for a while.

 * If the person she is interacting with leaves, show a frustrated
   emotion. Frustrated emotions are: sad, confused, recoil, surprised.

 * If someone else leaves, glance at that last location, or ignore
   the departure.

 * While interacting with someone:
   -- randomly display one of: happy, comprehending, engaged
   -- Ocasionally glance at other faces in the room.
   -- If the interaction has been long-running, then switch and pay
      attention to someone new

 * If the room is empty:
   -- showed bored expression (one of bored, sad, happy)
   -- look around the room
   -- If room still empty, yawn, blink-sleepy
   -- Go to sleep. Periodically wake. Wake gestures: shake-2, shake-3,
      blink


Resources:
----------
* Visual programming Language
  http://en.wikipedia.org/wiki/Visual_programming_language

* The current Hanson robotics Web motors user interface:
  https://github.com/hansonrobotics/ros_motors_webui

* Psi Theory
  http://en.wikipedia.org/wiki/Psi-Theory
