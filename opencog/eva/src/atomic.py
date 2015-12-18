#
# atomic.py - Simple atoms for simple Eva demo
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

import rospy
from ros_commo import EvaControl
from opencog.atomspace import AtomSpace, TruthValue
from opencog.bindlink import satisfaction_link
from opencog.type_constructors import *

from opencog.cogserver import get_server_atomspace

# The atomspace where everything will live.
atomspace = get_server_atomspace()
set_type_ctor_atomspace(atomspace)

# The ROS layer.
evl = EvaControl()

# Start RestAPI if it is available.
# For some reason crashes if started from cog terminal
try:
	from web.api.restapi import Start as RestAPI
	RestAPI().run(None, atomspace=atomspace)
except:
	print "Rest API can't be started. Check Python configuration"

# Global functions, because that's what PythonEval expects.
# Would be great if PythonEval was fixed to work smarter, not harder.
#
# Must return TruthValue, since EvaluationLinks expect TruthValues.

def do_look_left():
	evl.look_left()
	return TruthValue(1, 1)

def do_look_right():
	evl.look_right()
	return TruthValue(1, 1)

def do_wake_up():
	evl.wake_up()
	return TruthValue(1, 1)

def do_go_sleep():
	evl.go_sleep()
	return TruthValue(1, 1)

def glance_at_face(face_id_node):
	face_id = int(float(face_id_node.name))
	print "Python glance at face id", face_id
	evl.glance_at(face_id)
	return TruthValue(1, 1)

def look_at_face(face_id_node):
	face_id = int(float(face_id_node.name))
	# print "Python look at face id", face_id
	evl.look_at(face_id)
	return TruthValue(1, 1)

def gaze_at_face(face_id_node):
	face_id = int(float(face_id_node.name))
	print "Python gaze at face id", face_id
	evl.gaze_at(face_id)
	return TruthValue(1, 1)

def gaze_at_point(x_node, y_node, z_node):
	x = float(x_node.name)
	y = float(y_node.name)
	z = float(z_node.name)
	evl.gaze_at_point(x, y, z)
	return TruthValue(1, 1)

def look_at_point(x_node, y_node, z_node):
	x = float(x_node.name)
	y = float(y_node.name)
	z = float(z_node.name)
	evl.look_at_point(x, y, z)
	return TruthValue(1, 1)


def do_emotion(emotion_node, duration_node, intensity_node):
	emotion = emotion_node.name
	intensity = float(intensity_node.name)
	duration = float(duration_node.name)
	# print "Python emotion: ", emotion, " for ", duration, " int ", intensity
	evl.expression(emotion, intensity, duration)
	return TruthValue(1, 1)

def do_gesture(gesture_node, intensity_node, repeat_node, speed_node):
	gesture = gesture_node.name
	intensity = float(intensity_node.name)
	repeat = float(repeat_node.name)
	speed = float(speed_node.name)
	# print "Python gesture: ", gesture, ", int: ", intensity, \
	#	", rep: ", repeat, ", speed: ", speed
	evl.gesture(gesture, intensity, repeat, speed)
	return TruthValue(1, 1)

def explore_saccade():
	# print "Python: Explore Saccade"
	evl.explore_saccade()
	return TruthValue(1, 1)

def conversational_saccade():
	# print "Python: Conversational Saccade"
	evl.conversational_saccade()
	return TruthValue(1, 1)

def blink_rate(mean_node, var_node):
	mean = float(mean_node.name)
	var  = float(var_node.name)
	# print "Python: blink-rate: ", mean, " variation ", var
	evl.blink_rate(mean, var)
	return TruthValue(1, 1)


# Return true as long as ROS is running.
def ros_is_running():
	if (rospy.is_shutdown()):
		return TruthValue(0, 1)
	return TruthValue(1, 1)
