#
# put_atoms.py - Place atoms into atomspace.
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

from opencog.atomspace import AtomSpace
from opencog.scheme_wrapper import scheme_eval_h
from opencog.cogserver import get_server_atomspace

# Simple API to isolate opencog atoms and imports from the ROS code,
# so that we don't hack both ROS and Opencog in the same module.
# Why? Not sure, seems like a good idea.
#
class PutAtoms:

	def __init__(self):
		self.atomspace = get_server_atomspace()

	# Put a marker in the AtomSpace to indicate that the robot is
	# talking now.
	def chatbot_speech_start(self):
		scheme_eval_h(self.atomspace, "(State chat-state chat-talk)")

	def chatbot_speech_stop(self):
		scheme_eval_h(self.atomspace, "(State chat-state chat-listen)")

	# Put a marker in the StomSpace to indicate that the robot is
	# happy, enthsed about what its saying.
	def chatbot_affect_happy(self):
		scheme_eval_h(self.atomspace, "(State chat-affect chat-happy)")

	def chatbot_affect_negative(self):
		scheme_eval_h(self.atomspace, "(State chat-affect chat-negative)")
