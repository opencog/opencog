
Movement Interfaces
===================

This directory defines an API for controlling the robot physical
movements, including making facial expressions, controlling the
eye blink rates, looking at locations, and saying things with
some sort of vocal inflection.

The API publishes various ROS messages to drive the robot.
The current API assumes that there is some animation rig that
converts animations to actually motor-drive commands.  This is
currently `blender`; the blender animation API is defined in
[blender_api](https://github.com/hansonrobotics/blender_api).

The OpenCog behaviors interface to ROS by calling the functions in
`atomic.py`. This file is just a "thin" wrapper around the actual ROS
code, which is in `ros_commo.py`.  A non-ROS, debugging-only interface
is in `atomic-dbg.py`; it does not import ROS, and only prints to
stdout. It can be used for a text-only chatbot.
