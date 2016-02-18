
Notes
=====

The files in this directory are non-essential to the system; they're
just some random sketches, ideas and notes.

* `btree-demo.scm`: a super-simplified example of a behavior tree,
   implemented
* `general_behavior.py`: a copy of the original Owyl behavior tree.
* `behavior.cfg`: a copy of the original Owyl config file.
* `universal-fsm.scm`: a finite state machine


Some random notes about OpenPsi
-------------------------------

Demands
 * Entertain people
 * Make people smile
 * Crack jokes
 * Answer questions
 * Recognize people
 * Greet people
 * Inquire about their well-being

See Psi Theory http://en.wikipedia.org/wiki/Psi-Theory
See also Amen's implementation.

GUI Requirements
----------------
A GUI is needed to edit the behavior tree. Some notes about the
requirements immediately below.

Goal: Create a Web UI that will allow theatrical performance artists
to script theatrical performances for the Eva head.  An example of a
simple theatrical performance might be ''When a someone enters the room,
briefly glance their way, give a faint smile, then resume with the
previous activity.''

The output of the script should be sequences of ROS messages that
control the Eva head (turn, look, smile), as well as the timing, pitch
and modulation of speech.

Creating the above might be a relatively straight-forward programming
task, were it not for one additional requirement: the scripts should be
executable within the OpenCog AtomSpace framework. This is because we
want, as a long-range goal, for OpenCog to be able to control the robot
behavior itself, starting with more basic Psi-Theory urges and desires,
rather than having Eva be an animated puppet.

This means that the authoring tool needs to be able to work with not
just some convenient scripting language, but it should work very
explicitly with a certain OpenCog-based scripting language.

Visual Programming interfaces
-----------------------------
Visual programming Language
http://en.wikipedia.org/wiki/Visual_programming_language

Some possible visual programming tools:

===Yes
* JS bootstrap + JS Angular + stuff Vytas recommended

===Maybe
* statecharts.org (Yakindu)
* Snap!
  -- Pluses: web-based, AGPL
  -- Minuses: too focused on programming theory
* Scratch
* EToys -- based on squeak/smalltalk
* Python Notebooks?

===No
* Alice (for teaching programming)
* AgentSheets (proprietary)
* Flowgarithm (too low level)
* Hopscotch (proprietary)
* LARP (flowcharts, too low-level)
* Raptor (flowchart-based, too low-level, non-web)
* Visual Logic (proprietary, flowchart, non-web)
* ToonTalk (aimed at children, no web UI)
* StarLog (programming language, not visual)
* Cameleon (programming language, not visual)


Chat interfaces:
----------------
-- gtalk uses XMPP
   gtalk is going away

-- gplus hangouts is the new thing.
   * protocol is proprietary.
   * API is here: https://developers.google.com/+/hangouts/api/
     The API sucks.  It assumes ... all the wrong things. WTF, google.
   * https://github.com/tdryer/hangups/ but it seems to be chat only?
   * hangouts seems to use, maybe, use webRTC

-- blender: preferences->system
   frame server port 8080
   I can't figure out how to start the frame server.

-- screen-grab:
   * VLC is supposed to be able to do this, but its
     slow/compute-intensive

   * http://www.maartenbaert.be/simplescreenrecorder/
   * ffmpeg/libav can do live-streaming
     http://www.maartenbaert.be/simplescreenrecorder/live-streaming/
   * istanbul has a pipeline:  istximagesrc name=videosource display-name=:0.0 screen-num=0 use-damage=false show-pointer=false ! video/x-raw-rgb,framerate=10/1 ! videorate ! ffmpegcolorspace !  videoscale method=1 !  video/x-raw-yuv,width=1020,height=480,framerate=10/1 ! theoraenc !  oggmux name=mux ! filesink location=/tmp/tmprSI2tO
   * Just need to replace the filesink in last step with a pipe or
     char device ...

-- WebRTC
   * Complete demo here:
     https://bitbucket.org/webrtc/codelab
     http://www.html5rocks.com/en/tutorials/webrtc/basics/
   * screensharing w/ webrtc


Resources:
----------
* The current Hanson robotics Web motors user interface:
  https://github.com/hansonrobotics/ros_motors_webui
