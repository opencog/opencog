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

from opencog.scheme_wrapper import scheme_eval, scheme_eval_h, scheme_eval_as

# Simple API to isolate opencog atoms and imports from the ROS code,
# so that we don't hack both ROS and Opencog in the same module.
# Why? Not sure, seemed like a good idea at the time.
#
# How to unit-test:
# -----------------
# This class pokes atoms into an AtomSpace. It should be the same
# AtomSpace as the one that the rest of the system is using; in
# particular, it should be the one that the behavior tree is defined
# in. This can be manually tested like so:
#
#     pat = PutAtoms()
#     pat.btree_stop()
#     pat.btree_run()
#
# If everything is hooked up OK, then `pat.btree_run()` should start
# the btree running.  If the AtomSpace is borked, it will complain
# that some DefinedPredicateNode is not defined, and will do nothing at
# all (because, duhh, there's no DefinedPredicateNode in the bad
# AtomSpace). Since you only have a guile prompt, and not a python
# prompt, you'll need to actually do this:
#
#    (python-eval "from put_atoms import *")
#    (python-eval "pat = PutAtoms()\npat.btree_stop()")
#    (python-eval "pat = PutAtoms()\npat.btree_run()")
#
class PutAtoms:

	def __init__(self):
		# Get the atomspace that the scheme is using at just this moment.
		self.atomspace = scheme_eval_as('(cog-atomspace)')

		# Needed for the public define of chat-state, chat-start, etc.
		# XXX Except that this doesn't actually make chat-state visible?
		# WTF? But use-modules in btree.scm does work... strange.
		scheme_eval(self.atomspace, "(use-modules (opencog eva-model))")

	# Let atomspace know that vocalization has started or ended.
	def vocalization_started(self):
		scheme_eval_h(self.atomspace, "(State chat-state chat-start)")

	def vocalization_ended(self):
		scheme_eval_h(self.atomspace, "(State chat-state chat-stop)")

	# Indicate that the robot heard freindly speech
	def affect_happy(self):
		scheme_eval_h(self.atomspace, "(State chat-affect chat-happy)")

	# Indicate that the robot heard negative speech
	def affect_negative(self):
		scheme_eval_h(self.atomspace, "(State chat-affect chat-negative)")

	# Pass the text that STT heard into opencog.
	# XXX procees-query is not really the best API, here.
	# Must run in a new thread, else it deadlocks in python, since
	# the text processing results in python calls.
	def perceived_text(self, text):
		scheme_eval(self.atomspace,
			'(call-with-new-thread (lambda () (process-query "luser" "' + text + '")))')

	# Start or stop the behavior tree.
	def btree_stop(self):
		scheme_eval_h(self.atomspace, "(halt)")

	def btree_run(self):
		scheme_eval_h(self.atomspace, "(run)")
