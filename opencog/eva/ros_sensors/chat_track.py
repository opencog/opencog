#
# chat_track.py -  Misc chatbot message handling.
# Copyright (C) 2014,2015,2016  Hanson Robotics
# Copyright (C) 2015,2016 Linas Vepstas
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

import rospy
from chatbot.msg import ChatMessage
from atomic_msgs import AtomicMsgs

'''
Subscribe to text ROS messages, typically from the speech-to-text
subsystem, and pass these onwards into the cogserver.

Unit test by saying
    rostopic pub --once chatbot_speech std_msgs/String "Hello Sopha!"
'''

class ChatTrack:

	def __init__(self):
		self.atomo = AtomicMsgs()
		rospy.Subscriber("chatbot_speech", ChatMessage,
			self.chat_perceived_text_cb)

	# ---------------------------------------------------------------
	# Speech-to-text callback
	def chat_perceived_text_cb(self, msg):
		if msg.confidence >= 50:
			# XXX FIXME WTF Why are there two of these????
			# Surely one of these is enough to do the trick!
			self.atomo.who_said(msg.utterance)
			self.atomo.perceived_text(msg.utterance)
