This is a functional and nearly complete implementation.  A few
previously-supported features are missing, these are listed at the end.


Design overview
===============

* The main behavior tree is in `btree.scm`.  Many of the configurable
  parameters are in `behavior-cfg.scm`.

* The OpenCog behaviors interface to ROS by calling the functions in
  `atomic.py`. This file is just a "thin" wrapper around the actual ROS
  code, which is in `ros_commo.py`.  A non-ROS, debugging-only interface
  is in `atomic-dbg.py`; it does not import ROS, and only prints to
  stdout.

* The behavior tree works with visible faces beased on face ID's,
  and is only interested in the visible faces, and not thier locations.
  It outputs commands such as "look at face ID 42". The actualy tracking
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

* XXX The code currently uses a bogus design, where a "room state" is
  set, to be either "empty" or "non-empty".  This is not really needed,
  since one can also query if the number of visible faces is greater
  than zero, or not.  The room-state stuff is in `faces.scm` and should
  probably be removed.


Running
=======
The code can be run in debug mode, or in fully-integrated mode.

* Debug mode: Edit `btree.scm`, locate the line referencing `atomic.py`
  and change it to `atomic-dbg.py`.  The start guile, and, at the guile
  prompt, say `(load "btree.scm")`  ad then `(run)`.  See the top of
  the `btree.scm` file for more hints on debugging.

* Integrated mode:
  Change directory to your `catkin_ws` and then run `scripts/eva.sh`.
  This will start up a byobu multiplexed terminal, and will run ROS,
  the ros face tracker, the blender API and the opencog server
  automatically, and will launch most of the needed behavior scripts.


Debugging notes
===============
Cython modules are installed here:
```
`/usr/local/share/opencog/python/opencog`
```

Get to the cogserver prompt like this:
```
`rlwrap telnet localhost 17020`
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

Note that if the room state changes, `(show-room-state)` will show the
wrong state until after both cog-binds ar performed!

```
rostopic pub --once /opencog/glance_at std_msgs/Int32 29
rostopic pub --once /opencog/look_at std_msgs/Int32 29
rostopic pub --once /opencog/gaze_at std_msgs/Int32 29
```


Unfinished work TODO List
=========================
Grep for XXX in the code:

* Implement the face-study saccade.

* Implement the look-for-attention segment.

* Implement the remaining new behaviors from `public_ws`.

* Implement the non-uniform RandomChoice link.

Enhancement TODO List
=====================
A random list of ideas.

* Integrate with OpenPsi.

* Integrate the authoring GUI.

* Integrate the chatbot.

* Integrate face recognition.  This not only requires the API, but
  also needs SQL enabled to remember things.

* Use the Timeserver ... The timeserver tracks intervals, so that queries
  can be performed against a time-range.  It might make sense to redesign
  the timeserver to be native atomese. Anyway, none of the logic in the
  btree currently needs TimeServer functions.

* However, there is a fair amount of "did a unit of time elapse since
  event XYZ" logic in the btree.  Its kind-of klunky, and could use
  a better design.

* SatisfactionLink is kind-of not-needed; should be able to directly
  execute SequentialAndLink, SequentialOrLink. (when there are no
  actual query variables in it!?)
