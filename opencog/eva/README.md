Performance Scripting Interface
===============================
Experimental user interface for creating performance scripts to control
the Hanson Robotics Eva robot head.

Goal: Creeate a Web UI that will allow theatrical performance artists
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
This is version 0.0.0 of the project. Nothing is here yet. There's going
to be lots of miscellaneous experiments and a mish-mash of code.


Resources:
----------
* Visual programming Language
  http://en.wikipedia.org/wiki/Visual_programming_language

* The current Hanson robotics Web motors user interface:
  https://github.com/hansonrobotics/ros_motors_webui

* Psi Theory
  http://en.wikipedia.org/wiki/Psi-Theory
