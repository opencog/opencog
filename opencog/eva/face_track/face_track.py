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
import logging

from std_msgs.msg import Int32
from chatbot.msg import ChatMessage
from pi_face_tracker.msg import FaceEvent, Faces

from face_atomic import FaceAtomic

logger = logging.getLogger('hr.eva_behavior.face_track')

# Thin python wrapper, to subscribe to face-tracking ROS messages,
# (face ID's, 3D face locations) and then re-wrap these as opencog
# atoms, via FaceAtomic, and forward them on into the OpenCog
# space-time server.
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

		# The OpenCog API. This is used to send face data to OpenCog.
		self.atomo = FaceAtomic()

		# List of currently visible faces
		self.visible_faces = []

		# Subscribed pi_vision topics and events
		self.TOPIC_FACE_EVENT = "/camera/face_event"
		self.EVENT_NEW_FACE = "new_face"
		self.EVENT_LOST_FACE = "lost_face"
		self.EVENT_RECOGNIZED_FACE = "recognized_face"
		# Overrides current face being tracked by WebUI
		self.EVENT_TRACK_FACE = "track_face"

		self.TOPIC_FACE_LOCATIONS = "/camera/face_locations"

		# Face appearance/disappearance from pi_vision
		rospy.Subscriber(self.TOPIC_FACE_EVENT, FaceEvent, self.face_event_cb)

		# Face location information from pi_vision
		rospy.Subscriber(self.TOPIC_FACE_LOCATIONS, Faces, self.face_loc_cb)

		rospy.Subscriber("chatbot_speech", ChatMessage, self.stt_cb)

		rospy.Subscriber("/behavior_control", Int32, self.behavior_control_cb)

		# Control Eyes and face by default
		self.control_mode = 255

	# ---------------------------------------------------------------
	# Speech-to-text callback
	def stt_cb(self, msg):
		if msg.confidence >= 50:
			self.atomo.who_said(msg.utterance)

	# ----------------------------------------------------------
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

	# Adds given face to atomspace as requested face
	def track_face(self, faceid):
		if faceid in self.visible_faces:
			logger.info("Face requested interaction: " + str(faceid))
			self.atomo.add_tracked_face_to_atomspace(faceid)

	# ----------------------------------------------------------

	# pi_vision ROS callbacks

	# pi_vision ROS callback, called when a new face is detected,
	# or a face is lost.
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
	# location data for us. This happens frequently (about 10x/second)
	def face_loc_cb(self, data):
		if not self.control_mode & self.C_FACE_TRACKING:
			return

		for face in data.faces:
			# Update location of a face. The location is stored in the
			# OpenCog space server (octomap).
			if face.id in self.visible_faces:
				self.atomo.update_face_octomap(face.id,
				            face.point.x, face.point.y, face.point.z)


	# Enable/disable Opencog face-tracking.  This is driven by the
	# master control GUI.
	def behavior_control_cb(self, data):
		# Is facetracking currently enabled?
		facetracking = self.control_mode & self.C_FACE_TRACKING
		self.control_mode = data.data
		print("New Control mode %i" % self.control_mode )

		# If face-tracking was enabled, and is now disabled ...
		if facetracking > 0 and self.control_mode & self.C_FACE_TRACKING == 0:
			self.atomo.update_ft_state_to_atomspace(False)
			# Need to clear faces:
			for face in self.visible_faces[:]:
				self.remove_face(face)

		elif self.control_mode & self.C_FACE_TRACKING > 0:
			self.atomo.update_ft_state_to_atomspace(True)
