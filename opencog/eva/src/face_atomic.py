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


from opencog.atomspace import AtomSpace, TruthValue
from opencog.type_constructors import *

from opencog.cogserver import get_server_atomspace


# The code here is a quick, cheap hack to place information about
# visible faces into the cogserver atomspace.  There ae several
# different ways we could implement this:
#
# A) Open a socket to the cogserver, send atoms through the socket.
# B) Run inside the cogserver, and place atoms in the atomspace
#    directly.
#
# At the moment, for convenience, B) is implemented. However, A better
# long term choice might be design A).


class FaceAtomic:

	def __init__(self):
		# The atomspace where everything will live.
		self.asp = get_server_atomspace()
		set_type_ctor_atomspace(self.asp)

	# Add a face to the atomspace.
	def add_face_to_atomspace(self, faceid):
		self.define_face(faceid)
		print "Defined face in atomspace: ", faceid

	# Remove a face from the atomspace.
	def remove_face_from_atomspace(self, faceid):

		# AtomSpace remove takes handle as an argument.
		self.asp.remove(self.define_face(faceid))
		print "Removed face from atomspace: ", faceid
		# XXX TODO Remove the NumberNode as well, if no
		# one is using it.

	# Define a handle the represent a visible face.
	# Return the handle.
	def define_face(self, faceid):
		face = EvaluationLink(
			ConceptNode("visible face"),
			ListLink(
				NumberNode(str(faceid))))
		return face
