#
# ros_commo.py - ROS messaging module for OpenCog behaviors.
# Copyright (C) 2015  Hanson Robotics
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License v3 as
# published by the Free Software Foundation and including the exceptions
# at http://opencog.org/wiki/Licenses
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU Affero General Public License
# along with this program; if not, write to:
# Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.


import rospy
import roslib
import time

# Eva ROS message imports
from std_msgs.msg import String, Int32
from blender_api_msgs.msg import AvailableEmotionStates, AvailableGestures
from blender_api_msgs.msg import EmotionState
from blender_api_msgs.msg import SetGesture
from blender_api_msgs.msg import Target
from blender_api_msgs.msg import BlinkCycle
from blender_api_msgs.msg import SaccadeCycle


# This class publishes various ROS messages to various locations.
# Class members are meant to be invoked from within the cogserver.
# Note that it ONLY publishes, it does not listen; currently listening
# must be done in some other ROS node, which must then convert those
# messages to scheme commands, and netcat them to the cogserver.
class EvaControl():

	def step(self):
		print "step once"
		return not rospy.is_shutdown()

	def look_left(self):
		self.look_at_point(1.0, 0.5, 0.0)
		return

	def look_right(self):
		self.look_at_point(1.0, -0.5, 0.0)
		return

	def look_center(self):
		self.look_at_point(1.0, 0.0, 0.0)
		return

	def happy(self):
		# Create the message
		exp = EmotionState()
		exp.name = 'happy'
		exp.magnitude = 1.0
		exp.duration.secs = 5
		exp.duration.nsecs = 250*1000000
		self.emotion_pub.publish(exp)
		print "Just published: ", exp.name

	def sad(self):
		# Create the message
		exp = EmotionState()
		exp.name = 'sad'
		exp.magnitude = 1.0
		exp.duration.secs = 5
		exp.duration.nsecs = 250*1000000
		self.emotion_pub.publish(exp)
		print "Just published: ", exp.name

	def nod(self):
		ges = SetGesture()
		ges.name = 'nod-2'
		ges.magnitude = 1.0
		ges.repeat = False
		ges.speed = 1.0
		self.gesture_pub.publish(ges)
		print "Just published gesture: ", ges.name

	# ----------------------------------------------------------
	# Wrapper for emotional expressions
	def expression(self, name, intensity, duration):
		# Create the message
		exp = EmotionState()
		exp.name = name
		exp.magnitude = intensity
		exp.duration.secs = int(duration)
		exp.duration.nsecs = 1000000000 * (duration - int(duration))
		self.emotion_pub.publish(exp)
		print "Publish expression: ", exp.name

	# Wrapper for gestures
	def gesture(self, name, intensity, repeat, speed):
		# Create the message
		ges = SetGesture()
		ges.name = name
		ges.magnitude = intensity
		ges.repeat = repeat
		ges.speed = speed
		self.gesture_pub.publish(ges)
		print "Published gesture: ", ges.name

	# ----------------------------------------------------------
	# Look at, gaze at, glance at face id's
	# Look_at turns entire head in that direction, once.
	# Gaze_at has the eyes track the face location (servoing)
	# Glance_t is a momentary eye movement towards the face target.

	def look_at(self, face_id):
		print "----- Looking at face:" + str(face_id)
		self.look_at_pub.publish(face_id)

	def gaze_at(self, face_id):
		print "----- Gazing at face:" + str(face_id)
		self.gaze_at_pub.publish(face_id)

	def glance_at(self, face_id):
		print "----- Glancing at face:" + str(face_id)
		self.glance_at_pub.publish(face_id)

	# ----------------------------------------------------------
	# Explicit directional look-at, gaze-at locations

	# Turn only the eyes towards the given target point.
	# Coordinates: meters; x==forward, y==to Eva's left.
	def gaze_at_point(self, x, y, z):
		print "gaze at point: ", x, y, z

		trg = Target()
		trg.x = x
		trg.y = y
		trg.z = z
		self.gaze_pub.publish(trg)

	# Turn head towards the given target point.
	# Coordinates: meters; x==forward, y==to Eva's left.
	def look_at_point(self, x, y, z):
		print "look at point: ", x, y, z

		trg = Target()
		trg.x = x
		trg.y = y
		trg.z = z
		self.turn_pub.publish(trg)

	# ----------------------------------------------------------
	# Subscription callbacks
	# Get the list of available gestures.
	def get_gestures_cb(self, msg):
		print("Available Gestures:" + str(msg.data))

	# Get the list of available emotional expressions.
	def get_emotion_states_cb(self, msg):
		print("Available Emotion States:" + str(msg.data))

	def __init__(self):
		rospy.init_node("OpenCog_Eva")
		print("Starting OpenCog Behavior Node")

		rospy.Subscriber("/blender_api/available_emotion_states",
		       AvailableEmotionStates, self.get_emotion_states_cb)

		rospy.Subscriber("/blender_api/available_gestures",
		       AvailableGestures, self.get_gestures_cb)

		self.emotion_pub = rospy.Publisher("/blender_api/set_emotion_state",
		                                   EmotionState, queue_size=1)
		self.gesture_pub = rospy.Publisher("/blender_api/set_gesture",
		                                   SetGesture, queue_size=1)
		self.blink_pub = rospy.Publisher("/blender_api/set_blink_randomly",
		                                   BlinkCycle, queue_size=1)
		self.saccade_pub = rospy.Publisher("/blender_api/set_saccade",
		                                   SaccadeCycle, queue_size=1)

		self.affect_pub = rospy.Publisher("chatbot_affect_express",
		                                   EmotionState, queue_size=1)

		# XYZ coordinates of where to turn and look.
		self.turn_pub = rospy.Publisher("/blender_api/set_face_target",
			Target, queue_size=1)

		self.gaze_pub = rospy.Publisher("/blender_api/set_gaze_target",
			Target, queue_size=1)

		# Int32 faceid of the face to glence at or turn and face.
		self.glance_at_pub = rospy.Publisher("/opencog/glance_at",
			Int32, queue_size=1)

		self.look_at_pub = rospy.Publisher("/opencog/look_at",
			Int32, queue_size=1)

		self.gaze_at_pub = rospy.Publisher("/opencog/gaze_at",
			Int32, queue_size=1)
