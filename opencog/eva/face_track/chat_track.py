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

# Thin python wrapper, to subscribe to speech-to-text ROS messages,
# and then re-wrap these as OpenCog atoms, and forward them on into
# the OpenCog space-time server.
#
class ChatTrack:

	def __init__(self):

		# The OpenCog API. This is used to send face data to OpenCog.
		self.atomo = AtomicMsgs()

		rospy.Subscriber("chatbot_speech", ChatMessage, self.stt_cb)

	# ---------------------------------------------------------------
	# Speech-to-text callback
	def stt_cb(self, msg):
		if msg.confidence >= 50:
			self.atomo.who_said(msg.utterance)
