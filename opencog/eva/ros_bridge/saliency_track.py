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

# XXX defined in head/src/vision/ros_nmpt_saliency
from ros_nmpt_saliency.msg import targets

'''
	This implements a ROS node that subscribes to the `/nmpt_saliency_point`
	updates saliency
'''

class SaliencyTrack:
	def __init__(self):
		self.atomo = AtomicMsgs()
		rospy.Subscriber('/nmpt_saliency_point', targets, self.sal_cb)

	def sal_cb(self, data):
		loc = data.positions[0]
		z=-(loc.y*2.0-1.0)
		x=1.0
		y=-1.0*(loc.x*2.0-1.0)
		#print "locations x="+str(x)+" y="+str(y)+" z="+str(z)+"\n"
		self.atomo.saliency(x,y,z,data.degree)
