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
from std_msgs.msg import String
from blender_api_msgs.msg import AvailableEmotionStates, AvailableGestures
from blender_api_msgs.msg import EmotionState
from blender_api_msgs.msg import SetGesture

# Human face visibility tracker
from face_track import FaceTrack



class EventLoop():

	def step(self):
		print "step once"
		return not rospy.is_shutdown()

	def look_left(self):
		self.facetrack.look_at_point(1.0, 0.5, 0.0)
		return

	def look_right(self):
		self.facetrack.look_at_point(1.0, -0.5, 0.0)
		return

	def look_center(self):
		self.facetrack.look_at_point(1.0, 0.0, 0.0)
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

#   def glance_at(self):
#      face_id =
#      print "----- Glancing at face:" + str(face_id)
#      glance_seconds = 1
#      self.facetrack.glance_at_face(face_id, glance_seconds)


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

		self.facetrack = FaceTrack()
