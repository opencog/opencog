
Human Face Visual Servoing/Tracking
===================================

Design overview
===============
This is implements a quick-n-dirty ROS node to keep track of the
human faces visible in a scene.  It receives face-enter/leave events
from the webcam + pi_vision subsystem, as well as the current face
locations, in cooperation with ROS tf2 position database, which
actually holds the actual 3D positions. 

It turns around and publishes face-detected/face-lost events to the
OpenCog AtomSpace.  It listens for look-at messages from OpenCog.

This is a stand-alone ROS node only because of a simple, stupid reason:
it implements a form of imprecise visual servoing: when told to look
at a face, it will cause Eva to actively track that face as it moves
around in the scene.  This update needs to be done continously, i.e. at
least 3-5 times a second, and this behavior is currently too
real-time-ish, to difficult to bother with in the AtomSpace. Currently.
This may change in the future, as we get beyond the prototyping stage.
