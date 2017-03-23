#
# face_recog.py - Face Recognition
# Copyright (C) 2016  Hanson Robotics
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
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
# 02110-1301  USA

import rospy
from atomic_msgs import AtomicMsgs

from face_id.msg import f_id
from face_id.msg import faces_ids

# Push information about recognized faces into the atomspace.
#
# This listens to the `/camera/face_recognition` ROS topic. Note that
# some ofther face-id subsystem generates face-recognition messages
# to the `/camera/face_locations` topic, using a different message
# format. (See the `face_track.py` file).  I am not sure what subsystem
# publishes where, or why. XXX FIXME Figure out why tehre are two
# different face-recognition subsystems in use, document them, and
# standardize on the message formats used.
class FaceRecog:
	def __init__(self):
		self.atomo = AtomicMsgs()
		rospy.Subscriber('/camera/face_recognition', faces_ids, self.face_cb)

	def face_cb(self, data):
		for fc in data.faces:
			self.atomo.face_recognition(fc.id, fc.name);
