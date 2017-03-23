#
# affect.py - Emotional affect detected in spoken speech.
# Copyright (C) 2015, 2017  Hanson Robotics
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
    This implements a ROS node that subscribes to the
    `chatbot_affect_perceive` topic, and passes the perception
    to the cogserver. Currently, the perception uses Dave DeMaris'
    adaptation of advertizing data to extract whether the speaker
    is in a good mood, or is negative.
'''

class Affect:
	def __init__(self):
		self.atomo = AtomicMsgs()
		rospy.Subscriber("chatbot_affect_perceive", String,
			self.language_affect_perceive_cb)

	# The perceived emotional content of words spoken to the robot.
	# That is, were people being rude to the robot? Polite to it? Angry
	# with it?  We subscribe; there may be multiple publishers of this
	# message: it might be supplied by some linguistic-processing module,
	# or it might be supplied by a chatbot.
	#
	# emo is of type std_msgs/String
	def language_affect_perceive_cb(self, emo):
		print 'chatbot perceived affect class =' + emo.data
		rospy.loginfo('chatbot perceived affect class =' + emo.data)
		if emo.data == "happy":
			# behavior tree will use these predicates
			self.atomo.affect_happy()

		else:
			self.atomo.affect_negative()
