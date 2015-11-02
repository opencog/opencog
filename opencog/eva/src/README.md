This is a functional but incomplete prototype.  Certain previously-
supported features are missing, such as face-study saccades. A list
of other to-do items are at the bottom.

Design overview
===============

* The main behavior tree is in `btree.scm`.  Many of the configurable
  parameters are in `behavior-cfg.scm`.

* The OpenCog behaviors interface to ROS by calling the functions in
  `atomic.py`. This file is just a "thin" wrapper around the actual ROS
  code, which is in `ros_commo.py`.  A non-ROS, debugging-only interface
  is in `atomic-dbg.py`; it does not import ROS, ad only prints to
  stdout.

* The behavior tree works with visible faces beased on face ID's,
  and is only interested in the visible faces, ant not thier locations.
  It outputs commad such as "look at face ID 42". The actualy tracking
  of face locations, ad the visual servoing needed to maintain a
  steady, accurate gaze is in the `face_track` directory.

* The `face_track` directory contains code for visual servoing: it
  receives ROS messages about human face locations from the webcam
  + pi_vision subsystem.  It calls methods in `face_atomic.py` to
  poke face-ids (ID numbers) into the atomspace.

  A new face (for example, faceid 123) is indicated with this message:
  ```
   (EvaluationLink (PredicateNode "visible face")
             (ListLink (NumberNode "123")))
  ```
  See `face_track/README.md` for details.


Running
=======

* Change directory to your `catkin_ws` and then run `scripts/eva.sh`.
  This will start up a byobu multiplexed terminal, and will run ROS,
  the ros face tracker, the blender API and the opencog server
  automatically, and will launch most of the needed behavior scripts.


Development shortcuts
=====================
* Start guile in this directory, then from the guile prompt:
```
(add-to-load-path "/usr/local/share/opencog/scm")
(use-modules (opencog))
(use-modules (opencog cogserver))
(start-cogserver "../scripts/opencog.conf")
```
Then load the python code:
```
; echo -e "py\n" | cat - atomic.py |netcat localhost 17020

(system "echo \"py\\n\" | cat - atomic.py |netcat localhost 17020")
```
Then back at the guile prompt:
```
(load "btree.scm")
```

Debugging notes
===============

Cython modules are installed here:
```
/usr/local/share/opencog/python/opencog
```

Get to the cogserver prompt like this:
```
rlwrap telnet localhost 17020
```
and then enter the python interpreter by saying `py`, or the scheme
interpreter by saying `scm`.

From the python prompt, the following should list the python
opencog modules:
```
help('opencog')
```

Face tracking debug
===================
Print all visible faces in the atomspace:

```
rlwrap telnet localhost 17020
(cog-incoming-set (PredicateNode "visible face"))
(cog-bind chk-room-empty)
(cog-bind chk-room-non-empty)
(show-room-state)
```

Not that if the room state changes, `(show-room-state)` will show the
wrong state until after both cog-binds ar performed!

```
rostopic pub --once /opencog/glance_at std_msgs/Int32 29
rostopic pub --once /opencog/look_at std_msgs/Int32 29
rostopic pub --once /opencog/gaze_at std_msgs/Int32 29
```


Enhancement TODO List
=====================
A list of changes to the atomspace that could help with this:

* Use TimeNode for time; use the Timeserver as needed
* Create a CurrentTimeNode that returns the current time.  To store
  the current time, one would say this:
```
    (PutLink (EvaluationLink (PredicateNode "$timstamp") (VariableNode "$ts"))
       (CurrentTimeNode "now"))
```
* Create RandomNode for a uniform distribution 0 to 1 which
  can be used to evaluate to true/false by using GreaterThanLink.

* SatisfactionLink is kind-of not-needed; should be able to directly
  execute SequentialAndLink, SequentialOrLink. or maybe not ...

* fix cog-execute to avoid doing eager execution..!? e.g. for
  RandomChoiceLink ...? Or is this OK to leave as-is?
