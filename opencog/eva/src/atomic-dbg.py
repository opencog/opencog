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

def do_look_left():
	# evl.look_left()
	return TruthValue(1, 1)

def do_look_right():
	# evl.look_right()
	return TruthValue(1, 1)

def glance_at_face(face_id_node):
	face_id = int(face_id_node.name)
	print "Python glance at face id", face_id
	# evl.glance_at(face_id)
	return TruthValue(1, 1)

def look_at_face(face_id_node):
	face_id = int(face_id_node.name)
	print "Python look at face id", face_id
	# evl.look_at(face_id)
	return TruthValue(1, 1)

def gaze_at_face(face_id_node):
	face_id = int(face_id_node.name)
	print "Python gaze at face id", face_id
	# evl.gaze_at(face_id)
	return TruthValue(1, 1)

def do_smile(duration, intensity):
	print "Python smiling for ", duration, " at strength ", intensity
	return TruthValue(1, 1)

### Define an executable pattern
##satisfaction_handle = SatisfactionLink(
##	SequentialAndLink(
##		EvaluationLink(
##			# GroundedPredicateNode("py: evl.look_right"),
##			# GroundedPredicateNode("py: do_look_right"),
##			GroundedPredicateNode("py: do_look_left"),
##			ListLink()))).h
##
### See if the pattern can be satsified.  This should result
### in a ROS message being sent.
##result = satisfaction_link(atomspace, satisfaction_handle)
