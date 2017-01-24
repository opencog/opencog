#
# saliency_track.py - Sound energy and power.
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

logger = logging.getLogger('hr.eva_behavior.face_recog')

class FaceRecog:
	def __init__(self):
		self.atomo = AtomicMsgs()
		rospy.Subscriber('/camera/face_recognition', faces_ids, self.face_cb)

	def face_cb(self, data):
		for fc in data.faces:
			#print "locations x="+str(x)+" y="+str(y)+" z="+str(z)+"\n"
			self.atomo.face_id(fc.id,fc.name);
