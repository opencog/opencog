#! /usr/bin/env python
#
# main.py - Main entry point for face tracker
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
# You should have received a copy of the GNU Affero General Public
# License
# along with this program; if not, write to:
# Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.


import rospy
import time
from face_track import FaceTrack


print "Start face-tracking node"
ft = FaceTrack()

while not rospy.is_shutdown():
	ft.do_look_at_actions()
	time.sleep(0.1)

print "Exit face-tracking node"
