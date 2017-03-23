#
# atomic-dbg.py - OpenCog python schema debug wrapper.
#
# This is a wrapper for debugging the OpenCog code that controls the Eva
# blender model. It provides exactly the same GroundedPredicateNode functions
# as the normal API, but, instead of sending messages out on ROS, it simply
# prints to stdout.  Thus, its a stub, and can be used without starting up
# all of ROS and blender.  Handy for behavior debugging, as well as for
# vision-deprived, disembodied chatbot debugging.
#
# Copyright (C) 2015, 2016 Linas Vepstas
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

import time
from opencog.atomspace import TruthValue

# Global functions, because that's what PythonEval expects.
# Would be great if PythonEval was fixed to work smarter, not harder.
#
# Must return TruthValue, since EvaluationLinks expect TruthValues.
#
# The print messages are all parentheical, third-person; that's because
# they will typically be going out to some IRC channel, and should resemble
# captioning-for-the-blind, in this situation (or stage directions).
#
def prt_msg(face_id_node):
	face_id = int(face_id_node.name)
	print "Python face id", face_id
	return TruthValue(1, 1)

def do_wake_up():
	# evl.wake_up()
	print "(Eva wakes up)"
	return TruthValue(1, 1)

def do_go_sleep():
	# evl.go_sleep()
	print "(Eva falls asleep)"
	return TruthValue(1, 1)

def glance_at_face(face_id_node):
	face_id = int(float(face_id_node.name))
	print "(Eva glances at face id", face_id, ")"
	# evl.glance_at(face_id)
	return TruthValue(1, 1)

def look_at_face(face_id_node):
	now = time.time()
	if now - look_at_face.prev_time > 3 :
		look_at_face.prev_time = now
		face_id = int(float(face_id_node.name))
		print "(Eva looks at face id", face_id, ")"

	# evl.look_at(face_id)
	return TruthValue(1, 1)

look_at_face.prev_time = 0

def gaze_at_face(face_id_node):
	face_id = int(float(face_id_node.name))
	print "(Eva gazes at face id", face_id, ")"
	# evl.gaze_at(face_id)
	return TruthValue(1, 1)


# Moves eyes only, not entire head.
def gaze_at_point(x_node, y_node, z_node):
	x = float(x_node.name)
	y = float(y_node.name)
	z = float(z_node.name)

	# Plain-English description of the actions.
	if (y < -0.1):
		print "(Eva looks to the right)"
	elif (y > 0.1):
		print "(Eva looks to the left)"

	elif (-0.1 < y and y < 0.1 and -0.1 < z and z < 0.1):
		print "(Eva looks straight ahead)"

	if (z < -0.1):
		print "(Eva looks down)"
	elif (z > 0.1):
		print "(Eva looks up)"

	# print "(Eva gazes at point", x, y, z, ")"
	# evl.gaze_at_point(x, y, z)
	return TruthValue(1, 1)

# Turns entire head.
def look_at_point(x_node, y_node, z_node):
	x = float(x_node.name)
	y = float(y_node.name)
	z = float(z_node.name)

	# Plain-English description of the actions.
	if (y < -0.1):
		print "(Eva turns to the right)"
	elif (y > 0.1):
		print "(Eva turns to the left)"

	elif (-0.1 < y and y < 0.1 and -0.1 < z and z < 0.1):
		print "(Eva turns straight ahead)"

	if (z < -0.1):
		print "(Eva turns her face downwards)"
	elif (z > 0.1):
		print "(Eva turns her face upwards)"


	# print "(Eva looks at point", x, y, z, ")"
	# evl.look_at_point(x, y, z)
	return TruthValue(1, 1)

def do_face_expression(face_expression_node, duration_node, intensity_node):
	face_expression = face_expression_node.name
	duration = float(duration_node.name)
	intensity = float(intensity_node.name)
	print "(Eva expresses", face_expression, "facial expression for", duration, \
		 "seconds, with intensity", intensity, ")"

	# print "Python facial expression: ", face_expression, " for ", duration, " int ", intensity
	# evl.expression(face_expression, intensity, duration)
	return TruthValue(1, 1)

def do_gesture(gesture_node, intensity_node, repeat_node, speed_node):
	gesture = gesture_node.name
	intensity = float(intensity_node.name)
	repeat = float(repeat_node.name)
	speed = float(speed_node.name)
	# evl.gesture(gesture, intensity, repeat, speed)
	print "(Eva performs gesture:", gesture, ", intensity: ", intensity, \
		", repeat: ", repeat, ", speed: ", speed, ")"
	return TruthValue(1, 1)

def publish_behavior(event_node):
	print "(Behavior event:", event_node.name, ")"
	return TruthValue(1, 1)

def explore_saccade():
	print "(Eva switches to explore saccade)"
	# evl.explore_saccade()
	return TruthValue(1, 1)

def conversational_saccade():
	print "(Eva switches to conversational saccade)"
	# evl.conversational_saccade()
	return TruthValue(1, 1)

def listening_saccade():
	print "(Eva switches to listening saccade)"
	# evl.conversational_saccade()
	return TruthValue(1, 1)

def blink_rate(mean_node, var_node):
	mean = float(mean_node.name)
	var  = float(var_node.name)
	print "(Eva blink-rate: ", mean, " variation ", var, ")"
	# evl.blink_rate(mean, var)
	return TruthValue(1, 1)

def say_text(text_node):
	text = text_node.name
	print "(Eva says: ", text, ")"
	# evl.say_text(text)
	return TruthValue(1, 1)

# Return true as long as ROS is running.
def ros_is_running():
	# if (rospy.is_shutdown())
	#	return TruthValue(0, 1)
	return TruthValue(1, 1)
