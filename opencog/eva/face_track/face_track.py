#
# face_track.py -  Registery and tracking of visible human faces
# Copyright (C) 2014,2015  Hanson Robotics
# Copyright (C) 2015 Linas Vepstas
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
from pi_face_tracker.msg import FaceEvent, Faces
from blender_api_msgs.msg import Target

from face_atomic import FaceAtomic

# A Face. Currently consists only of an ID number, a 3D location,
# and the time it was last seen.  Should be extended to include
# the size of the face, possibly the location of the eyes, and,
# if possible, the name of the human attached to it ...
class Face:
	def __init__(self, fid, point):
		self.faceid = fid
		self.x = point.x
		self.y = point.y
		self.z = point.z
		self.t = time.time()

# A registery (in-memory database) of all human faces that are currently
# visible, or have been recently seen.  Implements various basic look-at
# actions, including:
# *) turning to face a given face
# *) tracking a face with the eyes
# *) glancing at a currrently-visible face, or at a location where a
#    face that was recently seen.
#
# It does this by subscribing to pi_vision topics, to get face ID's and
# 3D locations.  It maintains a database of 3D locations in ROS TF. It
# then sends face_id's to the CogServer, using the FaceAtomic class.
# These messages are currently sent as scheme strings.
#
# This class also subscribes to several /opencog topics, which is where
# the CogServer will send commands. These includes commands to stare
# at a given face_id, to glance at a face_id, and so on.  The handling
# is done here, and not in OpenCog, because right now, we don't want to
# track dynamic 3D locations in OpenCog. That is, we don't want to
# implement the analog of TF inside the CogServer.
#
# Upon receiving messages on /opencog, it then obeys these, and servos
# blender appropriately.
#
class FaceTrack:

	def __init__(self):

		rospy.init_node("OpenCog_Facetracker")
		print("Starting OpenCog Face Tracker ROS Node")

		# The OpenCog API. This is used to send face data to OpenCog.
		self.atomo = FaceAtomic()

		# List of currently visible faces
		self.visible_faces = []
		# List of locations of currently visible faces
		self.face_locations = {}

		# List of no longer visible faces, but seen recently.
		self.recent_locations = {}
		# How long (in seconds) to keep around a recently seen, but now
		# lost face. tf does the tracking for us.
		self.RECENT_INTERVAL = 20

		# Current look-at-target
		self.look_at = 0
		self.gaze_at = 0
		self.glance_at = 0
		self.first_glance = -1
		self.glance_howlong = -1

		# How often we update the look-at target.
		self.LOOKAT_INTERVAL = 1
		self.last_lookat = 0

		# Last time that the list of active faces was vacuumed out.
		self.last_vacuum = 0
		self.VACUUM_INTERVAL = 1

		# Subscribed pi_vision topics and events
		self.TOPIC_FACE_EVENT = "/camera/face_event"
		self.EVENT_NEW_FACE = "new_face"
		self.EVENT_LOST_FACE = "lost_face"

		self.TOPIC_FACE_LOCATIONS = "/camera/face_locations"

		# Subscribed OpenCog commands
		self.TOPIC_GLANCE_FACE = "/opencog/glance_at"
		self.TOPIC_LOOKAT_FACE = "/opencog/look_at"
		self.TOPIC_GAZEAT_FACE = "/opencog/gaze_at"
		rospy.Subscriber(self.TOPIC_GLANCE_FACE, Int, self.glance_at_face)
		rospy.Subscriber(self.TOPIC_LOOKAT_FACE, Int, self.look_at_face)
		rospy.Subscriber(self.TOPIC_GAZEAT_FACE, Int, self.gaze_at_face)

		# Published blender_api topics
		self.TOPIC_FACE_TARGET = "/blender_api/set_face_target"
		self.TOPIC_GAZE_TARGET = "/blender_api/set_gaze_target"

		# Face appearance/disappearance from pi_vision
		rospy.Subscriber(self.TOPIC_FACE_EVENT, FaceEvent, self.face_event_cb)

		# Face location information from pi_vision
		rospy.Subscriber(self.TOPIC_FACE_LOCATIONS, Faces, self.face_loc_cb)

		# Which face to look at
		# rospy.Subscriber(self.TOPIC_FACE_TARGET, xxxFaceEvent, xxxself.face_event_cb)

		# Where to look
		self.look_pub = rospy.Publisher(self.TOPIC_FACE_TARGET, \
			Target, queue_size=10)

		self.gaze_pub = rospy.Publisher(self.TOPIC_GAZE_TARGET, \
			Target, queue_size=10)

		# Frame in which coordinates will be returned from transformation
		self.LOCATION_FRAME = "blender"
		# Transform Listener.Allows history of RECENT_INTERVAL
		self.tf_listener = tf.TransformListener(False, rospy.Duration(self.RECENT_INTERVAL))

	# ---------------------------------------------------------------
	# Public API. Use these to get things done.

	# Turn only the eyes towards the given target face; track that face.
	def gaze_at_face(self, faceid):
		print ("gaze at: " + str(faceid))

		# Look at neutral position, 1 meter in front
		if 0 == faceid :
			trg = Target()
			trg.x = 1.0
			trg.y = 0.0
			trg.z = 0.0
			self.gaze_pub.publish(trg)

		self.last_lookat = 0
		if faceid not in self.visible_faces :
			self.gaze_at = 0
			return

		self.gaze_at = faceid

	# Turn entire head to look at the given target face. The head-turn is
	# performed only once per call; after that, the eyes will then
	# automatically track that face, but the head will not.  Call again,
	# to make the head move again.
	#
	def look_at_face(self, faceid):
		print ("look at: " + str(faceid))

		# Look at neutral position, 1 meter in front
		if 0 == faceid :
			trg = Target()
			trg.x = 1.0
			trg.y = 0.0
			trg.z = 0.0
			self.look_pub.publish(trg)

		self.last_lookat = 0
		if faceid not in self.visible_faces :
			self.look_at = 0
			return

		self.look_at = faceid

	def glance_at_face(self, faceid, howlong):
		print("glance at: " + str(faceid) + " for " + str(howlong) + " seconds")
		self.glance_at = faceid
		self.glance_howlong = howlong
		self.first_glance = -1

	# ---------------------------------------------------------------
	# Private functions, not for use outside of this class.

	# Start tracking a face
	def add_face(self, faceid):
		if faceid in self.visible_faces:
			return

		self.visible_faces.append(faceid)

		print("New face detected: " +
			str(self.visible_faces))
		self.atomo.add_face_to_atomspace(faceid)


	# Stop tracking a face
	def remove_face(self, faceid):
		self.atomo.remove_face_from_atomspace(faceid)

		print("Lost face; visibile faces now: " + str(self.visible_faces))
		if faceid in self.visible_faces:
			self.visible_faces.remove(faceid)


	# ----------------------------------------------------------
	# Main look-at action driver.  Should be called at least a few times
	# per second.  This publishes all of the eye-related actions that the
	# blender api robot head should be performing.
	#
	# This performs multiple actions:
	# 1) updates the list of currently visible faces
	# 2) updates the list of recently seen (but now lost) faces
	# 3) If we should be looking at one of these faces, then look
	#    at it, now.
	#
	# Note that step 3 is a kind-of imprecise visual-servoing. That is,
	# if the look-at face is moving around, the step 3 will result in
	# her automatically tracking that face, as it moves.
	def do_look_at_actions(self) :
		now = time.time()

		# Should we be glancing elsewhere? If so, then do it, and
		# do it actively, i.e. track that face intently.
		if 0 < self.glance_at:
			if self.first_glance < 0:
				self.first_glance = now
			if (now - self.first_glance < self.glance_howlong):
				face = None

				# Find latest postion known
				try:
					trg = self.face_target(self.glance_at)
					self.gaze_pub.publish(trg)
				except:
					print("Error: no face to glance at!")
					self.glance_at = 0
					self.first_flance = -1
			else :
				# We are done with the glance. Resume normal operations.
				self.glance_at = 0
				self.first_glance = -1

		# Publish a new lookat target to the blender API
		elif (now - self.last_lookat > self.LOOKAT_INTERVAL):
			self.last_lookat = now

			# Update the eye position, if need be. Skip, if there
			# is also a pending look-at to perform.

			if 0 < self.gaze_at and self.look_at <= 0:
				print("Gaze at id " + str(self.gaze_at))
				try:
					if not self.gaze_at in self.visible_faces:
						raise Exception("Face not visible")
					trg = self.face_target(self.gaze_at)
					self.gaze_pub.publish(trg)
				except tf.LookupException as lex:
					print("Warning: TF has forgotten about face id:" +
						str(self.look_at))
					self.remove_face(self.look_at)
					self.look_at_face(0)
					return
				except Exception as ex:
					print("Error: no gaze-at target: ", ex)
					self.gaze_at_face(0)
					return

			if 0 < self.look_at:
				print("Look at id: " + str(self.look_at))
				try:
					if not self.look_at in self.visible_faces:
						raise Exception("Face not visible")
					trg = self.face_target(self.look_at)
					self.look_pub.publish(trg)
				except tf.LookupException as lex:
					print("Warning: TF has forgotten about face id: " +
						str(self.look_at))
					self.remove_face(self.look_at)
					self.look_at_face(0)
					return
				except Exception as ex:
					print("Error: no look-at target: ", ex)
					self.look_at_face(0)
					return

				# Now that we've turned to face the target, don't do it
				# again; instead, just track with the eyes.
				self.gaze_at = self.look_at
				self.look_at = -1

	# ----------------------------------------------------------
	# pi_vision ROS callbacks

	# pi_vision ROS callback, called when a new face is detected,
	# or a face is lost.  Note: I don't think this is really needed,
	# the face_loc_cb accomplishes the same thing. So maybe should
	# remove this someday.
	def face_event_cb(self, data):
		if data.face_event == self.EVENT_NEW_FACE:
			self.add_face(data.face_id)

		elif data.face_event == self.EVENT_LOST_FACE:
			self.remove_face(data.face_id)

	# pi_vision ROS callback, called when pi_vision has new face
	# location data for us. Because this happens frequently (10x/second)
	# we also use this as the main update loop, and drive all look-at
	# actions from here.
	def face_loc_cb(self, data):
		for face in data.faces:
			fid = face.id
			loc = face.point
			# Sanity check.  Sometimes pi_vision sends us faces with
			# location (0,0,0). Discard these.
			if loc.x < 0.05:
				continue
			self.add_face(fid)

		# Now perform all the various looking-at actions
		self.do_look_at_actions()

	# Queries tf_listener to get latest available position
	# Throws TF exceptions if transform cannot be returned
	def face_target(self, faceid):
		(trans, rot) = self.tf_listener.lookupTransform( \
			self.LOCATION_FRAME, 'Face' + str(faceid), rospy.Time(0))
		t = Target()
		t.x = trans[0]
		t.y = trans[1]
		t.z = trans[2]
		return t
