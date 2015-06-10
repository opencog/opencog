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
class FaceAtomic:

	def __init__(self):
		self.hostname = "localhost"
		self.port = 17001

	# Add a face to the atomspace.
	def add_face_to_atomspace(self, faceid):
		face = self.define_face(faceid)
		netcat(self.hostname, self.port, face + "\n")
		print "Defined face in atomspace: ", faceid

	# Remove a face from the atomspace.
	def remove_face_from_atomspace(self, faceid):

		# AtomSpace cog-delete takes handle as an argument.
		face = self.delete_face(faceid)
		netcat(self.hostname, self.port, msg)
		print "Removed face from atomspace: ", faceid

	# Define simple string to define a face
	def define_face(self, faceid):
		face = "(EvaluationLink (PredicateNode \"visible face\") " + \
		       "(ListLink (NumberNode \"" + str(faceid) + "\")))\n"
		return face

	# Define commands to delete the face, and also to garbage-collect
	# the ListLink NumberNode.
	def delete_face(self, faceid):
		face = "(cog-delete " + \
		       "(EvaluationLink (PredicateNode \"visible face\") " + \
		       "(ListLink (NumberNode \"" + str(faceid) + "\"))))\n" + \
		       "(cog-delete " + \
		       "(ListLink (NumberNode \"" + str(faceid) + "\")))\n" +\
		       "(cog-delete (NumberNode \"" + str(faceid) + "\"))\n"
		return face
