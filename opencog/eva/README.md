Robot Performance Scripting
===========================
This repo contains performance, personality and self-awareness scripts
for controling the Hanson Robotics Eva robot emulator.  The emulator
is a Blender animation of a female head.  These scripts unite, in one
place, a visual subsystem, a chatbot, and an expressive face capable
of a wide range of emotional expressions and facial gestures.

The scripts are written in OpenCog "atomese", with the intent that this
enables integration with high-level cognitive, emotional and natural
language-processing functions.

The robot emulator is a Blender animation rig. It implements a dozen
facial expressions, another dozen gestures, such as blinking and
winking, as well as lip animations, for lip-syncing to speech. It can
be controlled through a set of ROS messages.  More info about this
robot is here:

* https://github.com/hansonrobotics/blender_api
* https://github.com/opencog/docker/tree/master/indigo/eva


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
being your freind and companion.


Current Architecture and Design
-------------------------------
The this time, the code here integrates three subsystems:

 * A visual subsystem, capable of detecting and tracking human faces
   visible in the room. These faces are localized in 3D space, and
   issued a numeric ID.

 * A collection of "behavior tree" scripts that react to people entering
   and leaving the room.  The scripts attempt to interact with the
   people who are visible, by displaying assorted facial expressions.

 * A representation model of the robot self and its surroundings (namely,
   the human faces visible in the room). The goal of this model is
   two-fold:

   ** Allow the robot to be self-aware, and engage in natural langauge
      dialog about what it is doing.

   ** Enable an "action orchestrator" to manage behaviors coming from
      multiple sources.

Some things it currently doesn't do, but should:

 * Provide much closer integration with chatbots.  Currently, the
   chatbot infrastructure is almost completely independent of this
   code. That's bad, and needs to be fixed.

   Such integration is a current focus of development, but is in the
   alpha stages.

 * Integrate superior face-tracking and face recognition tools.
   Right now, the face tracker eats too much CPU, and is completely
   unable to recognize known faces.

 * Have a GUI tools for editing behavior trees. The XXX tool has been
   suggested as such a tool.

 * Integration with OpenPsi behavior system.

 * Enable a memory, via the OpenCog AtomSpace database.  The goal here
   is to remember people and conversations and feelings, between
   power-offs and restarts.

 * Additional sensory systems and sensory inputs.  A perception
   synthesizer to coordinate all sensory input.

 * Have a much more sophisticated model of the world around it,
   including the humans in it. It should also have better model
   of itself, so that it understands its relationship to the world.


Summary of behaviors
--------------------
Below is a summary of the currently scripted behaviors.  Note that these
were diligently hand-crafted; it is meant to be only a rough draft for
a more sophisticated system that will incorporate more langauge
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
This is version 0.9.1 of the scripts. All of the old Owyl behavior
tree has been implemented.  Some initial work towards future goals
has been barely started.

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
