#
# face_atomic.py - Send face data to the cogserver/atomspace.
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

from netcat import netcat

# The code here is a quick, cheap hack to place information about
# visible faces into the cogserver atomspace. It opens a socket to
# the cogserver, and sends some atoms across.
#
# TODO: Send the face location as well.
#
class FaceAtomic:

	def __init__(self):
		self.hostname = "localhost"
		self.port = 17020

	def update_face_octomap(self, faceid, xx, yy, zz):
		face="(map-ato \"faces\" (NumberNode \""+str(faceid)+"\" (av 5 0 0)) "+str(xx)+" "+str(yy)+" "+str(zz)+")\n"
		netcat(self.hostname, self.port, face + "\n")
		#face = "(NumberNode \"" + str(faceid) + "\" (av 5 0 0))\n"
		#netcat(self.hostname, self.port, face + "\n")

	# def who_spoke(self,stt,strt,stp):
	# 	spoke = "(who-said? \""+stt+"\" "+str(strt)+" "+str(stp)+")\n"
	# 	print spoke
	# 	netcat(self.hostname, self.port, spoke + "\n")

	def who_said(self,stt):
		spoke = "(who-said? \""+stt+"\")\n"
		print spoke
		netcat(self.hostname, self.port, spoke + "\n")


	def save_snd1(self,x,y,z):
		#snd="(map-ato \"sounds\" (NumberNode \"1\") "+str(x)+" "+str(y)+" "+str(z)+")\n"
		snd="(map-sound "+str(x)+" "+str(y)+" "+str(z)+")\n"
		netcat(self.hostname, self.port, snd + "\n")

	def face_recognition(self,tracker_id,rec_id):
		'''
		tracker_id is id for tracking, needs to map to recocgzed or unrecognized rec_id.
		rec_id is "0" for un recognized face and some other string for recognized face
		if THIS evaluation link does not exist then face is seen but not determined as known or unknown
		rec_id is a string and is stored as ConceptNode
		tracker_id is integer stored as NumberNode
		'''
		stl = "(EvaluationLink (Predicate \"name\") (ListLink (ConceptNode \"" + str(tracker_id) + "\") (ConceptNode \"" + rec_id + "\")))\n"
		#stl = "(StateLink (NumberNode \"" + str(faceid) + "\") (ConceptNode \"" + rec_id +"\"))\n"
		netcat(self.hostname, self.port, stl + "\n")

	# Add a newly visible face to the atomspace.
	def add_face_to_atomspace(self, faceid):
		face = self.define_face(faceid)
		netcat(self.hostname, self.port, face + "\n")
		print "New visible face in atomspace: ", faceid

	# Focus attention on specific face.
	def add_tracked_face_to_atomspace(self, faceid):
		face = self.set_tracked_face(faceid)
		netcat(self.hostname, self.port, face + "\n")
		print "Force focus of attention on face: ", faceid

	# Remove a face (make it no longer visible).
	def remove_face_from_atomspace(self, faceid):

		# AtomSpace cog-delete takes handle as an argument.
		msg = self.delete_face(faceid)
		netcat(self.hostname, self.port, msg)
		print "Removed face from atomspace: ", faceid

	# Build a simple string to define a face
	def define_face(self, faceid):
		face = "(EvaluationLink (PredicateNode \"visible face\") " + \
		       "(ListLink (NumberNode \"" + str(faceid) + "\")))\n"
		#stl = "(StateLink (NumberNode \"" + str(faceid) + "\") (ConceptNode \"to-know\"))\n"
		return face

	# Build string to delete the face, and also to garbage-collect
	# the ListLink NumberNode.
	def delete_face(self, faceid):
		pattern = "(EvaluationLink (Predicate \"name\") (ListLink (ConceptNode \"" + str(faceid) + "\") (VariableNode \"f\")))"
		del_face = "(cog-execute! (PutLink (DeleteLink "+pattern+") (GetLink "+pattern+")))"
		face = "(cog-delete " + \
				"(EvaluationLink (PredicateNode \"visible face\") " + \
				"(ListLink (NumberNode \"" + str(faceid) + "\"))))\n" + \
				del_face + "\n" + \
				"(cog-delete " + \
				"(ListLink (NumberNode \"" + str(faceid) + "\")))\n" + \
				"(cog-delete (NumberNode \"" + str(faceid) + "\"))\n"
		return face

	# Build string to force attention to focus on the requested face.
	# This bypasses the normal "new face is visible" sequence, and
	# immediately shifts Eva's attention to this face.
	def set_tracked_face(self, faceid):
		face = '(StateLink request-eye-contact-state (NumberNode "' + \
		       str(faceid) + '"))'
		return face
	# Sets facetracking state in atomspace
	def update_ft_state_to_atomspace(self, ft_enabled):
		str = self.set_face_tracking_state(ft_enabled)
		netcat(self.hostname, self.port, str + "\n")

	# Build state string for facetracking state
	def set_face_tracking_state(self, enabled):
		if enabled:
			state = 'on'
		else:
			state = 'off'
		face = '(StateLink face-tracking-state face-tracking-%s)' % state
		return face
