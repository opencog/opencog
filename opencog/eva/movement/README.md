
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

Debugging notes
===============
Cython modules are installed here:
```
`/usr/local/share/opencog/python/opencog`
```

You can get a python command-line from the cogserver, like so:
```
`rlwrap telnet localhost 17020`
```
and then enter the python interpreter by saying `py`.  You can get
a scheme interpreter by saying `scm`.  You can telnet multiple times.
You can also call python from scheme, and scheme from python.

From the python prompt, the following should list the python
opencog modules:
```
help('opencog')
```

Verifying movements
===================

Assuming that `face_id` 29 is in the atomspace, then the robot
can turn to look at that face:

```
rostopic pub --once /opencog/glance_at std_msgs/Int32 29
rostopic pub --once /opencog/look_at std_msgs/Int32 29
rostopic pub --once /opencog/gaze_at std_msgs/Int32 29
```
