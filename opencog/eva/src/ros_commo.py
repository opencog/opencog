#
# ros_commo.py - ROS messaging module for OpenCog behaviors.
# Copyright (C) 2015  Hanson Robotics
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

import rosmsg
import rospy
import roslib
import time
import logging
import random
import yaml
import tf
import numpy
# Eva ROS message imports
from std_msgs.msg import String, Int32
from blender_api_msgs.msg import AvailableEmotionStates, AvailableGestures
from blender_api_msgs.msg import EmotionState
from blender_api_msgs.msg import SetGesture
from blender_api_msgs.msg import Target
from blender_api_msgs.msg import BlinkCycle
from blender_api_msgs.msg import SaccadeCycle
from blender_api_msgs.msg import SomaState
from chatbot.msg import ChatMessage
from dynamic_reconfigure.msg import Config

# Not everything has this message; don't break if it's missing.
# i.e. create a stub if its not defined.
#try:
#	from chatbot.msg import ChatMessage
#except (NameError, ImportError):
#	class ChatMessage:
#		def __init__(self):
#			self.utterance = ''
#			self.confidence = 0

from put_atoms import PutAtoms
logger = logging.getLogger('hr.OpenCog_Eva')

# ROS interfaces for the Atomese (OpenCog) Behavior Tree. Publishes
# ROS messages for animation control (smiling, frowning), and subscribes
# to STT/TTS and chatbot messages.
#
# This is meant to be a convenience wrapper, allowing Eva to be
# controlled from OpenCog Atomese.  Although it probably works as
# a stand-alone ROS node, it was not designed to be used that way.
# In particular, the python interpreter built into the atomspace
# will be runnig this code.
#
# It currently handles both control messages (publishing of expression
# and gesture animations), as well as some sensory input (mostly
# STT, TTS and chatbot interactions).  Visual servoing for face tracking
# is done by a stand-alone ROS node, in the face_tracker directory.
#
# This does listen to several topics that are used to turn behaviors on
# and off:
#
# `/behavior_switch`, which is used to start and stop the behavior tree.
#
# `/behavior_control`, which is used to enable/disable the publication
#      of classes of expression/gesture messages.
#
class EvaControl():

	# Control bitflags. Bit-wise anded with control_mode. If the bit
	# is set, then the corresponding ROS message is emitted, else it
	# is not.
	C_EXPRESSION = 1
	C_GESTURE = 2
	C_SOMA = 4
	C_SACCADE = 8
	C_EYES = 16
	C_FACE = 32

	def step(self):
		print "step once"
		return not rospy.is_shutdown()
    # Temporary disable sleeping.
	def go_sleep(self):
		# Vytas altered this in commit
		# 67ba02f75c5f82f4abb3e600711c97f65f007534
		# presumably because of conflicts with the current blender model!?
		# Or perhaps the behavior tree is sleeping too often?
		# self.soma_state('sleep', 1, 1, 3)
		# self.soma_state('normal', 0, 1, 0)
		self.soma_state('normal', 0.1, 1, 3)

	def wake_up(self):
		# self.soma_state('sleep', 0, 1, 0)
		self.soma_state('normal', 0.1, 1, 3)

	# ----------------------------------------------------------
	# Wrapper for emotional expressions
	def expression(self, name, intensity, duration):
		if 'noop' == name or (not self.control_mode & self.C_EXPRESSION):
			return
		# Create the message
		exp = EmotionState()
		exp.name = name
		exp.magnitude = intensity
		exp.duration.secs = int(duration)
		exp.duration.nsecs = 1000000000 * (duration - int(duration))
		self.emotion_pub.publish(exp)
		print "Publish expression:", exp.name

	# Wrapper for Soma state expressions
	def soma_state(self, name, intensity, rate, ease_in=0.0):
		if 'noop' == name or (not self.control_mode & self.C_SOMA):
			return
		# Create the message
		soma = SomaState()
		soma.name = name
		soma.magnitude = intensity
		soma.rate = rate
		soma.ease_in.secs = int(ease_in)
		soma.ease_in.nsecs = 1000000000 * (ease_in - int(ease_in))
		self.soma_pub.publish(soma)
		print "Publish soma state:", soma.name, "intensity:", intensity

	# Wrapper for gestures
	def gesture(self, name, intensity, repeat, speed):
		if 'noop' == name or (not self.control_mode & self.C_GESTURE):
			return
		# Create the message
		ges = SetGesture()
		ges.name = name
		ges.magnitude = intensity
		ges.repeat = repeat
		ges.speed = speed
		self.gesture_pub.publish(ges)
		print "Published gesture: ", ges.name

	# ----------------------------------------------------------
	# Look at, gaze at, glance at face id's
	# Look_at turns entire head in that direction, once.
	# Gaze_at has the eyes track the face location (servoing)
	# Glance_t is a momentary eye movement towards the face target.

	def look_at(self, face_id):
		# Can get called 10x/second, don't print.
		# print "----- Looking at face: " + str(face_id)
		if not self.control_mode & self.C_EYES:
			return
		self.look_at_pub.publish(face_id)

	def gaze_at(self, face_id):
		print "----- Gazing at face: " + str(face_id)
		self.gaze_at_pub.publish(face_id)

	def glance_at(self, face_id):
		print "----- Glancing at face: " + str(face_id)
		self.glance_at_pub.publish(face_id)

	# ----------------------------------------------------------
	# Explicit directional look-at, gaze-at locations

	# Turn only the eyes towards the given target point.
	# Coordinates: meters; x==forward, y==to Eva's left.
	def gaze_at_point(self, x, y, z):
		xyz1=numpy.array([x,y,z,1.0])
		xyz=numpy.dot(self.conv_mat,xyz1)
		trg = Target()
		trg.x = xyz[0]
		trg.y = xyz[1]
		trg.z = xyz[2]
		print "gaze at point: ", trg.x, trg.y, trg.z
		self.gaze_pub.publish(trg)

	# Turn head towards the given target point.
	# Coordinates: meters; x==forward, y==to Eva's left.
	def look_at_point(self, x, y, z):
		xyz1=numpy.array([x,y,z,1.0])
		xyz=numpy.dot(self.conv_mat,xyz1)
		trg = Target()
		trg.x = xyz[0]
		trg.y = xyz[1]
		trg.z = xyz[2]
		print "look at point: ", trg.x, trg.y, trg.z
		self.turn_pub.publish(trg)

	# ----------------------------------------------------------

	# Tell the world what we are up to. This is so that other
	# subsystems can listen in on what we are doing.
	def publish_behavior(self, event):
		print "----- Behavior pub: " + event
		self.behavior_pub.publish(event)

	# ----------------------------------------------------------
	# Wrapper for saccade generator.
	# This is setup entirely in python, and not in the AtomSpace,
	# as, at this time, there are no knobs worth twiddling.

	# Explore-the-room saccade when not conversing.
	# ??? Is this exploring the room, or someone's face? I'm confused.
	def explore_saccade(self):
		if not self.control_mode & self.C_SACCADE:
			return
		# Switch to conversational (micro) saccade parameters
		msg = SaccadeCycle()
		msg.mean =  0.8        # saccade_explore_interval_mean
		msg.variation = 0.3    # saccade_explore_interval_var
		msg.paint_scale = 0.3   # saccade_explore_paint_scale
		# From study face, maybe better default should be defined for
		# explore
		msg.eye_size = 15      # saccade_study_face_eye_size
		msg.eye_distance = 100  # saccade_study_face_eye_distance
		msg.mouth_width = 90    # saccade_study_face_mouth_width
		msg.mouth_height = 27  # saccade_study_face_mouth_height
		msg.weight_eyes = 0.8    # saccade_study_face_weight_eyes
		msg.weight_mouth = 0.2   # saccade_study_face_weight_mouth
		self.saccade_pub.publish(msg)

	# Used during conversation to study face being looked at.
	def conversational_saccade(self):
		if not self.control_mode & self.C_SACCADE:
			return
		# Switch to conversational (micro) saccade parameters
		msg = SaccadeCycle()
		msg.mean =  0.8         # saccade_micro_interval_mean
		msg.variation = 0.5     # saccade_micro_interval_var
		msg.paint_scale = 0.3   # saccade_micro_paint_scale
		#
		msg.eye_size = 11.5      # saccade_study_face_eye_size
		msg.eye_distance = 100 # saccade_study_face_eye_distance
		msg.mouth_width = 90    # saccade_study_face_mouth_width
		msg.mouth_height = 5  # saccade_study_face_mouth_height
		msg.weight_eyes = 0.8    # saccade_study_face_weight_eyes
		msg.weight_mouth = 0.2   # saccade_study_face_weight_mouth
		self.saccade_pub.publish(msg)

	# Used during conversation to study face being looked at.
	def listening_saccade(self):
		if not self.control_mode & self.C_SACCADE:
			return
		# Switch to conversational (micro) saccade parameters
		msg = SaccadeCycle()
		msg.mean =  1         # saccade_micro_interval_mean
		msg.variation = 0.6      # saccade_micro_interval_var
		msg.paint_scale = 0.3      # saccade_micro_paint_scale
		#
		msg.eye_size = 11        # saccade_study_face_eye_size
		msg.eye_distance = 80    # saccade_study_face_eye_distance
		msg.mouth_width = 50     # saccade_study_face_mouth_width
		msg.mouth_height = 13.0  # saccade_study_face_mouth_height
		msg.weight_eyes = 0.8    # saccade_study_face_weight_eyes
		msg.weight_mouth = 0.2   # saccade_study_face_weight_mouth
		self.saccade_pub.publish(msg)


	# ----------------------------------------------------------
	# Wrapper for controlling the blink rate.
	def blink_rate(self, mean, variation):
		msg = BlinkCycle()
		msg.mean = mean
		msg.variation = variation
		self.blink_pub.publish(msg)

	# ----------------------------------------------------------
	# Subscription callbacks
	# Get the list of available gestures.
	def get_gestures_cb(self, msg):
		print("Available Gestures:" + str(msg.data))

	# Get the list of available emotional expressions.
	def get_emotion_states_cb(self, msg):
		print("Available Emotion States:" + str(msg.data))

	# ----------------------------------------------------------

	# Tell the TTS subsystem to vocalize a plain text-string
	def say_text(self, text_to_say):
		rospy.logwarn('publishing text to TTS ' + text_to_say)
		self.tts_pub.publish(text_to_say)

	# The text that the STT module heard.
	# Unit test by saying
	#   rostopic pub --once perceived_text std_msgs/String "Look afraid!"
	#
	#def language_perceived_text_cb(self, text_heard):
	#	self.puta.perceived_text(text_heard.data)

	# The chat_heard message is of type chatbot/ChatMessage
	# from chatbot.msg import ChatMessage
	def chat_perceived_text_cb(self, chat_heard):
		if 'shut up' in chat_heard.utterance.lower():
			self.tts_control_pub.publish("shutup")
			return

		if chat_heard.confidence >= 50:
			self.puta.perceived_text(chat_heard.utterance)

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
			self.puta.vocalization_started()
		elif speech_event.data == "stop":
			rospy.loginfo("ending speech")
			self.puta.vocalization_ended()
		elif speech_event.data.startswith("duration"):
			rospy.loginfo(
                            "speech_event.data {}".format(speech_event.data))
		else:
			rospy.logerr("unknown speech_events message: " + speech_event.data)

	# Chatbot requests blink.
	def chatbot_blink_cb(self, blink):

		# XXX currently, this by-passes the OC behavior tree.
		# Does that matter?  Currently, probably not.
		rospy.loginfo(blink.data + ' says blink')
		blink_probabilities = {
			'chat_heard' : 0.4,
			'chat_saying' : 0.7,
			'tts_end' : 0.7 }
		# If we get a string not in the dictionary, return 1.0.
		blink_probability = blink_probabilities[blink.data]
		if random.random() < blink_probability:
			self.gesture('blink', 1.0, 1, 1.0)

	# The perceived emotional content of words spoken to the robot.
	# That is, were people being rude to the robot? Polite to it? Angry
	# with it?  We subscribe; there may be multiple publishers of this
	# message: it might be supplied by some linguistic-processing module,
	# or it might be supplied by an AIML-derived chatbot.
	#
	# emo is of type std_msgs/String
	def language_affect_perceive_cb(self, emo):
		rospy.loginfo('chatbot perceived emo class =' + emo.data)
		if emo.data == "happy":
			# behavior tree will use these predicates
			self.puta.affect_happy()

		else:
			self.puta.affect_negative()

		rospy.logwarn('publishing affect to chatbot ' + emo.data)
		self.affect_pub.publish(emo.data)

	# Turn behaviors on and off and set wholeshow configuration
	# Do not to clean visible faces as these can still be added/removed
	# while tree is paused
	def behavior_switch_callback(self, data):
		if data.data == "opencog_on":
			if not self.running:
				self.puta.wholeshow_start()
				self.running = True
		if data.data == "opencog_off":
			if self.running:
				self.puta.wholeshow_stop()
				self.look_at(0)
				self.gaze_at(0)
				self.running = False

	# Data is a bit-flag that enables/disables publication of messages.
	def behavior_control_callback(self, data):
		self.control_mode = data.data

	def openpsi_control_cb(self, data):
		"""
		This function is used for interactively modifying the weight of openpsi
		rules.
		"""
		param_list = yaml.load(rosmsg.get_yaml_for_msg(data.doubles + data.ints))
		for i in param_list:
			if i["name"] == "max_waiting_time":
				scm_str = '''(StateLink
				                 (AnchorNode "Chatbot: MaxWaitingTime")
				                 (TimeNode %f))''' % (i["value"])
			else:
				scm_str = '''(StateLink
				                 (ListLink
				                     (ConceptNode "OpenPsi: %s")
				                     (ConceptNode "OpenPsi: weight"))
				                 (NumberNode %f))''' % (i["name"], i["value"])

			self.puta.evaluate_scm(scm_str)

	#not needed anymore
	#def pub_snd_face_id(faceid):
	#	self.face_sound_pub.publish(faceid)

	def __init__(self):

		self.puta = PutAtoms()

		# The below will hang until roscore is started!
		rospy.init_node("OpenCog_Eva")
		print("Starting OpenCog Behavior Node")

		# ----------------
		# Get the available animations
		rospy.Subscriber("/blender_api/available_emotion_states",
		       AvailableEmotionStates, self.get_emotion_states_cb)

		rospy.Subscriber("/blender_api/available_gestures",
		       AvailableGestures, self.get_gestures_cb)

		# Send out facial expressions and gestures.
		self.emotion_pub = rospy.Publisher("/blender_api/set_emotion_state",
		                                   EmotionState, queue_size=1)
		self.gesture_pub = rospy.Publisher("/blender_api/set_gesture",
		                                   SetGesture, queue_size=1)
		self.soma_pub = rospy.Publisher("/blender_api/set_soma_state",
		                                   SomaState, queue_size=2)
		self.blink_pub = rospy.Publisher("/blender_api/set_blink_randomly",
		                                   BlinkCycle, queue_size=1)
		self.saccade_pub = rospy.Publisher("/blender_api/set_saccade",
		                                   SaccadeCycle, queue_size=1)

		# ----------------
		# XYZ coordinates of where to turn and look.
		self.turn_pub = rospy.Publisher("/blender_api/set_face_target",
			Target, queue_size=1)

		self.gaze_pub = rospy.Publisher("/blender_api/set_gaze_target",
			Target, queue_size=1)

		# Int32 faceid of the face to glence at or turn and face.
		self.glance_at_pub = rospy.Publisher("/opencog/glance_at",
			Int32, queue_size=1)

		self.look_at_pub = rospy.Publisher("/opencog/look_at",
			Int32, queue_size=1)

		self.gaze_at_pub = rospy.Publisher("/opencog/gaze_at",
			Int32, queue_size=1)

		#method shifted
		#self.face_sound_pub = rospy.Publisher("/manyyears/face_id",
		#	Int32, queue_size=1)


		# ----------------
		rospy.logwarn("setting up chatbot affect perceive and express links")

		# Publish cues to the chatbot, letting it know what we are doing.
		self.behavior_pub = rospy.Publisher("robot_behavior",
		                                  String, queue_size=1)

		# Tell the TTS subsystem what to vocalize
		# self.tts_pub = rospy.Publisher("tts", String, queue_size=1)
		# self.tts_pub = rospy.Publisher("/robot/chatbot_responses", String, queue_size=1)
		self.tts_pub = rospy.Publisher("chatbot_responses", String, queue_size=1)

		# Tell the chatbot what sort of affect to apply during
		# TTS vocalization. (Huhh???)
		self.affect_pub = rospy.Publisher("chatbot_affect_express",
		                                  String, queue_size=1)

		# Used to stop the vocalization.
		self.tts_control_pub = rospy.Publisher("tts_control",
		                                  String, queue_size=1)

		# String text of what the robot heard (from TTS)
		#no longer needed as chatbot_speech is Used
		#chatbot ai.py no longer usd so not published
		#rospy.Subscriber("perceived_text", String,
		#	self.language_perceived_text_cb)

		# Chat infrastructure text.
		rospy.Subscriber("chatbot_speech", ChatMessage,
			self.chat_perceived_text_cb)

		# Emotional content of words spoken to the robot.
		rospy.Subscriber("chatbot_affect_perceive", String,
			self.language_affect_perceive_cb)

		# Chatbot can request blinks correlated with hearing and speaking.
		rospy.Subscriber("chatbot_blink", String, self.chatbot_blink_cb)

		# Receive messages that indicate that TTS (or chatbot) has started
		# or finished vocalizing.
		rospy.Subscriber("speech_events", String, self.speech_event_cb)

		# ----------------
		# chatbot-psi controls
		rospy.Subscriber("/opencog_control/parameter_updates", Config,
			self.openpsi_control_cb)

		# ----------------
		# Boolean flag, turn the behavior tree on and off (set it running,
		# or stop it)
		rospy.Subscriber("/behavior_switch", String, \
			self.behavior_switch_callback)

		# Bit-flag to enable/disable publication of various classes of
		# expressions and gestures.
		rospy.Subscriber("/behavior_control", Int32, \
			self.behavior_control_callback)

		# Frame in which coordinates will be returned from transformation
		self.LOCATION_FRAME = "blender"
		# Transform Listener. Tracks history for RECENT_INTERVAL.
		self.tf_listener = tf.TransformListener()
		try:
			self.tf_listener.waitForTransform('camera', 'world', \
				rospy.Time(0), rospy.Duration(3.0))
		except Exception:
			print("No camera transforms!")
			exit(1)
		(trans,rot) = self.tf_listener.lookupTransform( \
			self.LOCATION_FRAME, 'camera', rospy.Time(0))
		a=tf.listener.TransformerROS()
		self.conv_mat=a.fromTranslationRotation(trans,rot)
		# Full control by default
		self.control_mode = 255
		self.running = True


# ----------------------------------------------------------------
