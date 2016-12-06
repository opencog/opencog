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
from face_track import FaceTrack

print "Start face-tracking node"
ft = FaceTrack()

# Check if sound-localization is being used
# TODO: move audio & sound related logic to sound directory.
try:
    ft.sl_matrix
    print "Sound localization is enabled"
except AttributeError:
    print "Sound localization is disabled"

rospy.spin()
print "Exit face-tracking node"
