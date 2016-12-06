#
# face_track.py -  Registery and tracking of visible human faces
# Copyright (C) 2014,2015,2016  Hanson Robotics
# Copyright (C) 2015,2016 Linas Vepstas
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

import time

import rospy
import tf
import math
import logging

from std_msgs.msg import Int32, String
from chatbot.msg import ChatMessage
from pi_face_tracker.msg import FaceEvent, Faces
from blender_api_msgs.msg import Target

from face_atomic import FaceAtomic
from geometry_msgs.msg import PoseStamped #for sound

logger = logging.getLogger('hr.eva_behavior.face_track')

# A registery (in-memory database) of all human faces that are currently
# visible, or have been recently seen.
#
# It does this by subscribing to pi_vision topics, to get face ID's and
# 3D locations.  It maintains a database of 3D locations in ROS TF. It
# then sends face_id's to the CogServer, using the FaceAtomic class.
# These messages are currently sent as scheme strings.
#
# This class also subscribes to several /opencog topics, which is where
# the CogServer sends commands. These includes commands to turn and
# face a given face_id, to (momentarily) glance at a face_id, and so on.
# The handling is done here, and not in OpenCog, because right now, we
# don't want to track dynamic 3D locations in OpenCog. That is, we don't
# want to implement the analog of TF inside the CogServer.
#
# XXX The above is not longer true!!! FIXME
#
# Upon receiving messages on /opencog, it then obeys these, and servos
# blender appropriately.
#
class FaceTrack:

	# Control flags. Ideally, FaceTrack should publish targets using
	# ros_commo EvaControl class.
	C_EYES = 16
	C_FACE = 32
	# Face tracking will be disabled if neither of these flags are set.
	# (this allows for a manual over-ride of face-tracking by other
	# control processes.)
	C_FACE_TRACKING = C_FACE | C_EYES

	def __init__(self):

		rospy.init_node("OpenCog_Facetracker")
		logger.info("Starting OpenCog Face Tracker ROS Node")

		# The OpenCog API. This is used to send face data to OpenCog.
		self.atomo = FaceAtomic()

		# List of currently visible faces
		self.visible_faces = []

		# How long (in seconds) to keep around a recently seen, but now
		# lost face. tf does the tracking for us.
		self.RECENT_INTERVAL = 20

		# Subscribed pi_vision topics and events
		self.TOPIC_FACE_EVENT = "/camera/face_event"
		self.EVENT_NEW_FACE = "new_face"
		self.EVENT_LOST_FACE = "lost_face"
		self.EVENT_RECOGNIZED_FACE = "recognized_face"
		# Overrides current face being tracked by WebUI
		self.EVENT_TRACK_FACE = "track_face"

		self.TOPIC_FACE_LOCATIONS = "/camera/face_locations"

		self.control_mode = 255

		# Face appearance/disappearance from pi_vision
		rospy.Subscriber(self.TOPIC_FACE_EVENT, FaceEvent, self.face_event_cb)

		# Face location information from pi_vision
		rospy.Subscriber(self.TOPIC_FACE_LOCATIONS, Faces, self.face_loc_cb)

		rospy.Subscriber("chatbot_speech", ChatMessage, self.stt_cb)

		# Frame in which coordinates will be returned from transformation
		self.LOCATION_FRAME = "blender"
		# Transform Listener. Tracks history for RECENT_INTERVAL.
		self.tf_listener = tf.TransformListener(False, \
		                        rospy.Duration(self.RECENT_INTERVAL))

		rospy.Subscriber("/behavior_control", Int32, \
			self.behavior_control_callback)
		# Control Eeys and face by default
		#self.control_mode = 255

		# Sound localization
		parameter_name = "sound_localization/mapping_matrix"
		if rospy.has_param(parameter_name):
			self.sl_matrix = rospy.get_param(parameter_name)
			rospy.Subscriber("/manyears/source_pose", PoseStamped, \
				self.sound_cb)

	# ---------------------------------------------------------------
	# Private functions, not for use outside of this class.

	def sound_cb(self, msg):
		self.store_sound_pos(msg.pose.position.x,
		                     msg.pose.position.y,
		                     msg.pose.position.z)

	# Store the location of the strongest sound-source in the
	# OpenCog space server.  This data arrives at a rate of about
	# 30 Hz, currently, from ManyEars.
	def store_sound_pos(self, x, y, z):
		# Convert to camera coordinates, using an affine matrix
		# (which combines a rotation and translation).
		#
		# A typical sl_matrix looks like this:
		#
		#   0.943789   0.129327   0.304204 0.00736024
		#   -0.131484   0.991228 -0.0134787 0.00895614
		#   -0.303278 -0.0272767   0.952513  0.0272001
		#   0          0          0          1
		#

		vs = [x, y, z, 1]
		r = [0, 0, 0, 0]
		for i in range(0,3):
			for j in range(0,3):
				r[i] += self.sl_matrix[i][j] * vs[j]

		self.atomo.save_snd1(r[0], r[1], r[2])

	# Start tracking a face
	def add_face(self, faceid):
		if faceid in self.visible_faces:
			return

		self.visible_faces.append(faceid)

		logger.info("New face added to visibile faces: " +
			str(self.visible_faces))
		self.atomo.add_face_to_atomspace(faceid)


	# Stop tracking a face
	def remove_face(self, faceid):
		self.atomo.remove_face_from_atomspace(faceid)

		if faceid in self.visible_faces:
			self.visible_faces.remove(faceid)

		logger.info("Lost face; visibile faces now: " + str(self.visible_faces))

	# ----------------------------------------------------------

	# Adds given face to atomspace as requested face
	def track_face(self, faceid):
		if faceid in self.visible_faces:
			logger.info("Face requested interaction: " + str(faceid))
			self.atomo.add_tracked_face_to_atomspace(faceid)
		return

	# ----------------------------------------------------------

	# Speech-to-text callback
	def stt_cb(self, msg):
		if msg.confidence >= 50:
			self.atomo.who_said(msg.utterance)

	# pi_vision ROS callbacks

	# pi_vision ROS callback, called when a new face is detected,
	# or a face is lost.  Note: I don't think this is really needed,
	# the face_loc_cb accomplishes the same thing. So maybe should
	# remove this someday.
	def face_event_cb(self, data):
		if not self.control_mode & self.C_FACE_TRACKING:
			return

		if data.face_event == self.EVENT_NEW_FACE:
			self.add_face(data.face_id)

		elif data.face_event == self.EVENT_LOST_FACE:
			self.remove_face(data.face_id)

		elif data.face_event == self.EVENT_TRACK_FACE:
			self.track_face(data.face_id)

		elif data.face_event == self.EVENT_RECOGNIZED_FACE:
			self.atomo.face_recognition(data.face_id, data.recognized_id)

	# pi_vision ROS callback, called when pi_vision has new face
	# location data for us. Because this happens frequently (10x/second)
	# we also use this as the main update loop, and drive all look-at
	# actions from here.
	def face_loc_cb(self, data):
		if not self.control_mode & self.C_FACE_TRACKING:
			return

		for face in data.faces:
			# Update location of a face. The location is stored in the
			# OpenCog space server (octomap).
			if face.id in self.visible_faces:
				self.atomo.update_face_octomap(face.id,
				            face.point.x, face.point.y, face.point.z)


	def behavior_control_callback(self, data):
		# Were there facetracking enabled
		facetracking = self.control_mode & self.C_FACE_TRACKING
		self.control_mode = data.data
		print("New Control mode %i" % self.control_mode )
		if facetracking > 0 and self.control_mode & self.C_FACE_TRACKING == 0:
			self.atomo.update_ft_state_to_atomspace(False)
			# Need to clear faces:
			for face in self.visible_faces[:]:
				self.remove_face(face)
		elif self.control_mode & self.C_FACE_TRACKING > 0:
			self.atomo.update_ft_state_to_atomspace(True)
