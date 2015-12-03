Robot Performance Scripting
===========================
This repo contains an OpenCog-based behavior-tree performance script that
controls the Hanson Robotics Sophia robot.  More info about this robot is
here:

* https://github.com/hansonrobotics/blender_api
* https://github.com/opencog/docker/tree/master/indigo/eva

The code here is a port of the previous `eva_behavior` Owyl python code.
There are multiple reasons to port this code to OpenCog, and away from
python:

* Behaviors can be dynamically edited, via a GUI tool (in developmet).
  By contrast, a Owyl-based design would have required editing and
  reloading python code.

* Integration with the Opencog Chatbot and the OpenPsi behaviors becomes
  easier.

* Sophia can have a memory between power-off's and restarts, since the
  OpenCog AtomSpace contents can be saved to a database.

Summary of behaviors
--------------------
Here's a summary of the currently scripted behaviors.  Note that these
were diligently hand-crafted; it is meant to be only a rough draft for
what will evetually be controlled by OpenPsi, including automatically
learned behaviors.  Thus, the scripts here really are just "temporary
scaffolding", to simplify initial phases of integration with OpenCog.

 * If the scene was empty and someone arrives, Eva interacts with the
   new arrival. Start by showing a 'surprised' or 'excited' expression.

 * If Eva is currently interacting with someone, and someone else
   arrives, she will either (dice-roll): glance at the newcomer or
   ignore them.   The probability of a glance increases if she has
   been interacting for a while.

 * If the person she is interacting with leaves, show a frustrated
   emotion. Frustrated emotions include: sad, confused, recoil, surprised.

 * If someone else leaves, glance at that last location, or ignore
   the departure.

 * While interacting with someone:
   -- Randomly display one of: happy, comprehending, engaged
   -- Occasionally glance at other faces in the room.
   -- If the interaction has been long-running, then switch and pay
      attention to someone new

 * If the room is empty:
   -- Show bored expression (one of bored, sad, happy)
   -- Look around the room, seeking attention.
   -- If the room is still empty, yawn, blink-sleepy, and go to sleep.
   -- Periodically wake. Wake gestures: shake-2, shake-3, blink.

Status
------
This is version 0.8.0 of the scripts. The basic behavior tree is
implemented. Some brand-new stuff as not bee integrated.  Some parts
are still klunky. Future plans are not started.

The `notes` directory contains a copy of the original owyl code,
(in `general_behavior.py`, for line-by-line reference), as well
as some other sketches and ideas.

Running
-------
* Use the `scripts/eva.sh` file, after adjusting paths in it for your
  installation. Or use the below:

* Start the webcam, pi_vision, and tf2 tracking nodes as usual.
* Start the Eva blender node, as usual.
* Load the scripts shown in scripts/eva.sh
* Start the opencog face-tracking node in the `face_track` directory.


GUI Requirements
----------------
A GUI is needed to edit the beavior tree. Some notes about the
requiremets immediately below.

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
executable within the OpenCog AtomSpace framework. This is because we
want, as a long-range goal, for OpenCog to be able to control the robot
behavior itself, starting with more basic Psi-Theory urges and desires,
rather than having Eva be an animated puppet.

This means that the authoring tool needs to be able to work with not
just some convenient scripting language, but it should work very
explicitly with a certain OpenCog-based scripting language.


Owyl implementation
-------------------
The previous implementation used Owyl trees to represent behaviors.
This choice turned out to be terrible: there was no way to do any
run-time editing of the behaviors; they are hard-coded in python code.
In particular, it's impossible to create a GUI or API for editing these,
as that would require editing python code, ad the recompiling it!

The old code can be found here:
`https://github.com/hansonrobotics/eva_behavior`


Resources:
----------
* Visual programming Language
  http://en.wikipedia.org/wiki/Visual_programming_language

* The current Hanson robotics Web motors user interface:
  https://github.com/hansonrobotics/ros_motors_webui

* Psi Theory
  http://en.wikipedia.org/wiki/Psi-Theory

TODO
----
* face_track.py has some new code for both face study, and current-face.
  merge this code.
