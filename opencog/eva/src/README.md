
Design overview
===============

* The main behavior tree launcher is in `btree-eva.scm`.  The
  configurable parameters are in `cfg-eva.scm`.

* There are two scheme modules: `eva-model`, which implements a model
  of the external world, and also a self-model of the robot.  This
  module allows the robot to be "aware" of it's surroundings, and to be
  "self-aware" during speech acts and during reasonsing.  The second
  module is `eva-behavior`, which implements the personality and the
  behaviors.

* The `eva-model` module has several components. The `faces` component
  models the visibile huan faces in the external environment. The
  `orchestrate` component is a "behavior multiplexer", resolving
  conflicting instructions for the robot to peform some action. The
  `self-model` component implements a model of what the robot is
  supposedly doing right now (so that the robot can be asked, e.g. "Are
  you smiling right now?")

* The `eva-behavior` module has several components.  The `express`
  component consists of commands to make facial expressions and
  gestures.  The `behavior` component contains the primary, full
  behavior tree that contols the entire performance.

* The OpenCog behaviors interface to ROS by calling the functions in
  `atomic.py`. This file is just a "thin" wrapper around the actual ROS
  code, which is in `ros_commo.py`.  A non-ROS, debugging-only interface
  is in `atomic-dbg.py`; it does not import ROS, and only prints to
  stdout. It can be used for a text-only chatbot.

* The behavior tree works with visible faces based on face ID's,
  and is only interested in the visible faces, and not thier locations.
  It outputs commands such as "look at face ID 42". The actual tracking
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

* Various behavior labels are published to the `robot_behavior` ROS
  topic, so that other ROS nodes can know what we are doing.  The
  published messages are just strings, and they are rather totally
  ad-hoc.  Grep for `(DefinedPredicate "Publish behavior")` to see
  these.  A random sampling includes:

    + `"Searching for attention"` -- Room is empty, can't see anyone.
    + `"This is boring"` -- No one is visible, no sound, we are bored.
    + `"Sound of crickets"` -- Haven't heard anything for a while.
    + `"Falling asleep"` -- Bored too long, no one visible, no sound.
    + `"Waking up"` -- Saw someone, heard something, slept too long.
    + `"Look at new arrival"` -- Look at newly-arrived person.
    + `"Look at requested face"` -- Handle WebGUI request.
    + `"Someone left"` -- Previously visible face no longer visible.
    + `"Interact with someone else" -- Change the focus of attention.
    + `"Who is there?" -- no one visible, but heard sound.
    + `"What was that sound?" -- Woken up by some sound.

* You can talk to her by using ROS messages. Some examples:
```
    rostopic pub --once perceived_text std_msgs/String "Pretend you're happy!"
    rostopic pub --once perceived_text std_msgs/String "look sad"
    rostopic pub --once perceived_text std_msgs/String "Emote saddness"
```


* XXX The code currently has a large variety of conficting and poor
  design choices in it -- its in a state of morphing from "OK so-so code"
  to "slightly better than before".  As a result of this hacking, various
  parts are being re-designed,  and bits of old, poor design still
  infest the code, and new featues are incomplete and half-working.
  It will be a good, long while before the code here settles down.


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

You can get a python command-line from the cogserver, like so:
```
`rlwrap telnet localhost 17020`
```
and then enter the python interpreter by saying `py`.  YOu can get
a scheme interpreter by saying `scm`.  You can telnet multiple times.
You can also call python from scheme, and scheme from python.

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

Enhancement TODO List
=====================
A list of random ideas.

* Replace the vision processing code by something better.

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

--------

* "Search for attention" is a state, need to record that in a state
  variable. Ditto for "Interact with people"

  (DefinedPredicateNode "Is interacting with someone?")

  Everythig emitted to ROS is a state.... look for teh behavior pubs.
