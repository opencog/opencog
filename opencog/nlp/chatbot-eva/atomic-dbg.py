#
# atomic-dbg.py - Simple atoms for simple Eva demo
# Copyright (C) 2015  Linas Vepstas
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


from opencog.atomspace import AtomSpace, TruthValue
from opencog.bindlink import satisfaction_link
from opencog.type_constructors import *

from opencog.cogserver import get_server_atomspace

###############################################################
# XXX FIXME This is a copy of the code in
# ros-behavior-scripting/src/atomic-dbg.py
# That code makes calls to ROS, to get the robot to actually
# do things. This code prints responses back to the user.
# It should be integrated, so that either ROS messages, or
# printed feedback, or both are generated.
###############################################################

# The atomspace where everything will live.
atomspace = get_server_atomspace()
set_type_ctor_atomspace(atomspace)

# Global functions, because that's what PythonEval expects.
# Would be great if PythonEval was fixed to work smarter, not harder.
#
# Must return TruthValue, since EvaluationLinks expect TruthValues.

def prt_msg(face_id_node):
	face_id = int(face_id_node.name)
	print "Python face id", face_id
	return TruthValue(1, 1)

def do_wake_up():
	# evl.wake_up()
	print "Python wake up"
	return TruthValue(1, 1)

def do_go_sleep():
	# evl.go_sleep()
	print "Python go to sleep"
	return TruthValue(1, 1)

def glance_at_face(face_id_node):
	face_id = int(float(face_id_node.name))
	print "Python glance at face id", face_id
	# evl.glance_at(face_id)
	return TruthValue(1, 1)

def look_at_face(face_id_node):
	face_id = int(float(face_id_node.name))
	print "Python look at face id", face_id
	# evl.look_at(face_id)
	return TruthValue(1, 1)

def gaze_at_face(face_id_node):
	face_id = int(float(face_id_node.name))
	print "Python gaze at face id", face_id
	# evl.gaze_at(face_id)
	return TruthValue(1, 1)


# Moves eyes only, not entire head.
def gaze_at_point(x_node, y_node, z_node):
	x = float(x_node.name)
	y = float(y_node.name)
	z = float(z_node.name)

	# Plain-English description of the actions.
	if (y < 0):
		print "(Eva looks to the right)"
	elif (y > 0):
		print "(Eva looks to the left)"

	if (z < 0):
		print "(Eva looks down)"
	elif (z > 0):
		print "(Eva looks up)"

	# print "Python gaze at point", x, y, z
	# evl.gaze_at_point(x, y, z)
	return TruthValue(1, 1)

# Turns entire head.
def look_at_point(x_node, y_node, z_node):
	x = float(x_node.name)
	y = float(y_node.name)
	z = float(z_node.name)

	# Plain-English description of the actions.
	if (y < 0):
		print "(Eva turns to the right)"
	elif (y > 0):
		print "(Eva turns to the left)"

	if (z < 0):
		print "(Eva turns her face downwards)"
	elif (z > 0):
		print "(Eva turns her face upwards)"


	# print "Python look at point", x, y, z
	# evl.look_at_point(x, y, z)
	return TruthValue(1, 1)

def do_emotion(emotion_node, duration_node, intensity_node):
	emotion = emotion_node.name
	duration = float(duration_node.name)
	intensity = float(intensity_node.name)
	# evl.expression(emotion, intensity, duration)
	print "Python emotion: ", emotion, " for ", duration, " int ", intensity
	return TruthValue(1, 1)

def do_gesture(gesture_node, intensity_node, repeat_node, speed_node):
	gesture = gesture_node.name
	intensity = float(intensity_node.name)
	repeat = float(repeat_node.name)
	speed = float(speed_node.name)
	# evl.gesture(gesture, intensity, repeat, speed)
	print "Python gesture: ", gesture, ", int: ", intensity, \
		", rep: ", repeat, ", speed: ", speed
	return TruthValue(1, 1)

def explore_saccade():
	print "Python: Explore Saccade"
	# evl.explore_saccade()
	return TruthValue(1, 1)

def conversational_saccade():
	print "Python: Conversational Saccade"
	# evl.conversational_saccade()
	return TruthValue(1, 1)

def blink_rate(mean_node, var_node):
	mean = float(mean_node.name)
	var  = float(var_node.name)
	print "Python: blink-rate: ", mean, " variation ", var
	# evl.blink_rate(mean, var)
	return TruthValue(1, 1)

# Return true as long as ROS is running.
def ros_is_running():
	# if (rospy.is_shutdown())
	#	return TruthValue(0, 1)
	return TruthValue(1, 1)
