
Design overview
===============

* The main behavior tree launcher is in `btree-eva.scm`.  The
  configurable parameters are in `cfg-eva.scm`.

* There are two scheme modules: `eva-model`, which implements a model
  of the external world, and also a self-model of the robot.  This
  module allows the robot to be "aware" of it's surroundings, and to be
  "self-aware" during speech acts and during reasoning.  The second
  module is `eva-behavior`, which implements the personality and the
  behaviors.

* The `eva-model` module has several components. The `faces` component
  models the visible human faces in the external environment. The
  `orchestrate` component is a "behavior multiplexer", resolving
  conflicting instructions for the robot to perform some action. The
  `self-model` component implements a model of what the robot is
  supposedly doing right now (so that the robot can be asked, e.g. "Are
  you smiling right now?")

* The `eva-behavior` module has several components.  The `express`
  component consists of commands to make facial expressions and
  gestures.  The `behavior` component contains the primary, full
  behavior tree that controls the entire performance.

* The behavior tree works with visible faces based on face ID's,
  and is only interested in the visible faces, and not their locations.
  It outputs commands such as "look at face ID 42". The actual tracking
  of face locations, ad the visual servoing needed to maintain a
  steady, accurate gaze is in the `face_track` directory.

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
    rostopic pub --once perceived_text std_msgs/String "Shake your head!"
    rostopic pub --once perceived_text std_msgs/String "look sad"
    rostopic pub --once perceived_text std_msgs/String "Emote sadness"
    rostopic pub --once perceived_text std_msgs/String "Turn to the left"
```


* XXX The code currently has a large variety of conflicting and poor
  design choices in it -- its in a state of morphing from "OK so-so code"
  to "slightly better than before".  As a result of this hacking, various
  parts are being re-designed,  and bits of old, poor design still
  infest the code, and new features are incomplete and half-working.
  It will be a good, long while before the code here settles down.

Face tracking debug
===================
Print all visible faces in the AtomSpace:

```
rlwrap telnet localhost 17020
(cog-incoming-set (PredicateNode "visible face"))
(cog-execute! chk-room-empty)
(cog-execute! chk-room-non-empty)
(show-room-state)
```

Note that if the room state changes, `(show-room-state)` will show the
wrong state until after both cog-execute's are performed!


Enhancement TODO List
=====================
A list of random ideas.

* Replace the vision processing code by something better.

* Integrate the authoring GUI.

* Integrate the chatbot.

* Integrate face recognition.  This not only requires the API, but
  also needs SQL enabled to remember things.

* Use the TimeServer ... The TimeServer tracks intervals, so that queries
  can be performed against a time-range.  It might make sense to redesign
  the TimeServer to be native atomese. Anyway, none of the logic in the
  btree currently needs TimeServer functions.

* However, there is a fair amount of "did a unit of time elapse since
  event XYZ" logic in the btree.  Its kind-of clunky, and could use
  a better design.

* SatisfactionLink is kind-of not-needed; should be able to directly
  execute SequentialAndLink, SequentialOrLink. (when there are no
  actual query variables in it!?)

--------

* "Search for attention" is a state, need to record that in a state
  variable. Ditto for "Interact with people"

  (DefinedPredicateNode "Is interacting with someone?")

  Everything emitted to ROS is a state.... look for the behavior pubs.

----------------

Pending bugs:
* (DefinedPredicateNode "Did someone leave?")  being called much much
   too often!!! ... why???
