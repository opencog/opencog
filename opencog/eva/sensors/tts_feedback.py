#
# tts_feedback.py - TTS (text to speech) feedback.
# Copyright (C) 2016,2017  Hanson Robotics
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
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
# 02110-1301  USA

import rospy
from std_msgs.msg import String
from atomic_msgs import AtomicMsgs

'''
    This implements a ROS node that subscribes to the `speech_events`
    topic, and passes these perceptions back to the cogserver.
    Currently, this is used to tell the cogserver when the TTS module
    has started, and finished vocalizing. That is, we sent it a
    sentence; we just want to know when it is actually saying it.
'''

class TTSFeedback:
	# Receive messages that indicate that TTS (or chatbot) has started
	# or finished vocalizing.
	def __init__(self):
		self.atomo = AtomicMsgs()
		rospy.Subscriber("speech_events", String, self.speech_event_cb)

	# Notification from text-to-speech (TTS) module, that it has
	# started, or stopped vocalizing.  This message might be published
	# by either the TTS module itself, or by some external chatbot.
	#
	#    rostopic pub --once speech_events std_msgs/String start
	#    rostopic pub --once speech_events std_msgs/String stop
	def speech_event_cb(self, speech_event):
		print('speech_event, type ' + speech_event.data)
		if speech_event.data == "start":
			rospy.loginfo("starting speech")
			self.atomo.vocalization_started()
		elif speech_event.data == "stop":
			rospy.loginfo("ending speech")
			self.atomo.vocalization_ended()
		elif speech_event.data.startswith("duration"):
			rospy.loginfo("speech_event.data {}".format(speech_event.data))
		else:
			rospy.logerr("unknown speech_events message: " + speech_event.data)
