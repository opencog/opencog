#
# room_brightness.py - Sound energy and power.
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

# XXX defined in head/src/vision/room_luminance/msg
from room_luminance.msg import Luminance

'''
    This implements a ROS node that subscribes to the `/opencog/room_luminance`
    sets brightness

'''

class RoomBrightness:
	def __init__(self):
		self.atomo = AtomicMsgs()
		rospy.Subscriber('/opencog/room_luminance', Luminance, self.bright_cb)

	def bright_cb(self, data):
		self.atomo.room_brightness(data.value)
