Robot Performance Scripting
===========================
This repo contains performance, personality and self-awareness scripts
for controlling the Hanson Robotics Eva robot emulator.  The emulator
is a Blender animation of a female head.  These scripts unite, in one
place, a visual subsystem, an audio subsystem, a chatbot, and an
expressive face capable of a wide range of emotional expressions and
facial gestures.

The scripts are written in OpenCog "atomese", with the intent that this
enables integration with high-level cognitive, emotional and natural
language-processing functions.  The scripts are in active development;
new designs and design proposals are actively debated on the mailing
list.

The robot emulator is a Blender animation rig. It implements a dozen
facial expressions, another dozen gestures, such as blinking and
winking, as well as lip animations, for lip-syncing to speech. It can
be controlled through a set of ROS messages.  More info about this
robot is here:

* https://github.com/hansonrobotics/blender_api
* https://github.com/opencog/docker/tree/master/indigo/eva

This directory also contains an assortment of ROS nodes that subscribe
to ROS visual and audio sensory inputs, and forward these to the
opencog spactime server (performing the needed format conversion).


Design Goals
------------
The code here is intended to provide the highest-level integration of
all of Eva's cognitive, language, vision, memory, personality and
behavioral functions.  It is meant to be the place where "everything
comes together".  As such, it is currently rather provisional, crude,
and conceptually flawed. It is also a focus of development.

The goal is to have a robot that one can talk to, interact with, and
have fun with. The robot should be smart: it should be able to recognize
you, learn things about you, and remember them.  It should be capable of
being your friend and companion.


Current Architecture and Design
-------------------------------
The this time, the code here integrates three subsystems:

 * Several ROS nodes that forward visual and sound data to the
   OpenCog spacetime server. This includes 3D locations of visible
   faces, the names of any recognized faces (as recognized by some
   external software), the direction from which sounds are coming
   from, and audio-power samples.

   (This needs to be replaced by a (much) better visual system.)

 * A collection of behavior rules that react to people entering
   and leaving the room, react to sounds, etc. The scripts attempt
   to interact with the people who are visible, by displaying
   assorted facial expressions.

   (This needs to be replaced by a library of selections, as described
   in [README-affects.md](README-affects.md).

 * A representation model of the robot self and its surroundings (namely,
   the human faces visible in the room). The goal of this model is
   two-fold:

   * Allow the robot to be self-aware, and engage in natural language
     dialog about what it is doing.

   * Enable an "action orchestrater" to manage behaviors coming from
     multiple sources.

Some things it currently doesn't do, but should:

 * Provide much closer integration with chatbots.  Currently, the
   chatbot infrastructure is almost completely independent of this
   code. That's bad, and needs to be fixed.

   Such integration is a current focus of development, but is in the
   alpha stages.

 * Integrate superior face-tracking and face recognition tools.
   Right now, the face tracker is recognizes known faces only with
   difficulty.

 * Integration with OpenPsi behavior system. However, see also the
   [affects proposal](README-affects.md), which is almost(?) more
   important(?)

 * Enable memory, via the OpenCog AtomSpace database.  The goal here
   is to remember people and conversations and feelings, between
   power-offs and restarts.  This requires changes to this repo,
   and also writing tools and utilities to simplify the SQL and/or
   file-dump management.

 * Additional sensory systems and sensory inputs.  A perception
   synthesizer to coordinate all sensory input. High priority:

  ++ Audio power envelope (half-done, see `sensors/audio_power.py`),
     fundamental frequency (of voice), rising/falling tone.
     Background audio power. Length of silent pauses.  Detection
     of applause, laughter, loud voices in the background, loud
     bangs.

  ++ Video-chaos: Is it light or dark? Is there lots of random
     motion in the visual field, or are things visually settled?

 * Have a much more sophisticated model of the world around it,
   including the humans in it. It should also have better model
   of itself, so that it understands its relationship to the world.


Summary of behaviors
--------------------
Below is a summary of the currently scripted behaviors.  Note that these
were diligently hand-crafted; it is meant to be only a rough draft for
a more sophisticated system that will incorporate more language
functions, a better model of the world around it, and itself, a better
model of emotional state, as well as automatic learning of new
behaviors.

 * If the scene was empty and someone arrives, Eva interacts with the
   new arrival. Start by showing a 'surprised' or 'excited' expression.

 * If Eva is currently interacting with someone, and someone else
   arrives, she will either (dice-roll): glance at the newcomer or
   ignore them.   The probability of a glance increases if she has
   been interacting for a while.

 * If the person she is interacting with leaves, show a frustrated
   emotion. Frustrated emotions include: sad, confused, recoil,
   surprised.

 * If someone else leaves, glance at that last location, or ignore
   the departure.

 * While interacting with someone:

   -- Randomly display one of: happy, comprehending, engaged.

   -- Occasionally glance at other faces in the room.

   -- If the interaction has been long-running, then switch and pay
      attention to someone new.

 * If the room is empty:

   -- Show bored expression (one of bored, sad, happy).

   -- Look around the room, seeking attention.

   -- If the room is still empty, yawn, blink-sleepy, and go to sleep.

   -- Periodically wake. Wake gestures: shake-2, shake-3, blink.

Status
------
This is version 0.9.2 of the scripts. All of the old Owyl behavior
tree has been implemented.  Some initial work towards future goals
has been barely started.

The `notes` directory contains a copy of the original Owyl code,
(in `general_behavior.py`, for line-by-line reference), as well
as some other sketches and ideas.

Running
-------
* Change directory to `catkin_ws`
* Use the `scripts/eva.sh` file, after adjusting paths in it for your
  installation. Or use the below:

* Start the webcam, and `pi_vision` nodes as usual.
* Start the Eva blender node, as usual.
* Load the scripts shown in `scripts/eva.sh`
* Start the ros-sensory-input node `main.py` in the `sensors` directory.


Unit Testing
------------
There is a simple test driver in `src/unit-test.scm`. It emulates the
appearance and disappearance of two faces every few seconds(!). It also
emulates chat-start and chat-stop.  It does not need or use the regular
face-tracker or vision system nor the chatbot.  If everything is
working, the blender head should wake up, smile, react appropriately,
and then fall asleep every 2 minutes or so.  This test runs in an
infinite loop.

TODO
----
* stage mode

* Search for and address the XXX's that appear in the various files.

* Blender currently published this:
```
  rostopic echo /blender_api/available_emotion_states
  data: ['irritated', 'happy', 'recoil', 'surprised', 'sad', 'confused',
  'worry', 'bored', 'engaged', 'amused', 'comprehending', 'afraid']
```
but we are not actually using all of these at this time ...

* Frustrated emotions are dialed down so much, they are invisible.
  The following seems to have no effect:
```
  (cog-evaluate! (DefinedPredicateNode "Show frustrated expression"))
```

* Need major overhaul of the time-space server API's. Need to be able
  to query them with pattern matcher -- need to create time-query atoms
  -- need to move atom factory to the classserver. Need to place
  sound-source direction into the space-server. (i.e. currently
  in time-map.scm map-sound)
