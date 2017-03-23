#
# general_behavior.py -  The primary Owyl behavior tree
# Copyright (C) 2014  Hanson Robotics
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

# system imports
import copy
import os
import random
import time

# tool imports
import owyl
from owyl import blackboard
import rospy
import roslib
import ConfigParser
import csv

# message imports
from std_msgs.msg import String
from blender_api_msgs.msg import AvailableEmotionStates, AvailableGestures
from blender_api_msgs.msg import EmotionState
from blender_api_msgs.msg import SetGesture
from blender_api_msgs.msg import Target

# local stuff.
from face_track import FaceTrack

# Basic holder for emotion-expression properties and probabilities
class Emotion:
	def __init__(self, name) :
		self.name = name
		self.probability = 0.0
		self.min_intensity = 0.0
		self.max_intensity = 1.0
		self.min_duration = 5.0
		self.max_duration = 15.0

# Basic holder for gesture properties and probabilities
class Gesture:
	def __init__(self, name):
		self.name = name
		self.probability = 0.0
		self.min_intensity = 0.0
		self.max_intensity = 1.0
		self.min_repeat = 0.0
		self.max_repeat = 1.0
		self.min_speed = 0.0
		self.max_speed = 1.0

class Tree():
	# ---------
	# Config File utilities
	def unpack_config_emotions(self, config, emo_class) :

		def get_values(from_config, num_values):
			rtn_values = [float(z.strip()) for z in from_config.split(",")]
			if len(rtn_values) != num_values:
				raise Exception("List lengths don't match!")
			return rtn_values

		names = [x.strip() for x in config.get("emotion", emo_class).split(",")]
		numb = len(names)

		probs = get_values(config.get("emotion", \
			emo_class + "_probabilities"), numb)
		mins = get_values(config.get("emotion", \
			emo_class + "_intensity_min"), numb)
		maxs = get_values(config.get("emotion", \
			emo_class + "_intensity_max"), numb)

		dins = get_values(config.get("emotion", \
			emo_class + "_duration_min"), numb)
		daxs = get_values(config.get("emotion", \
			emo_class + "_duration_max"), numb)

		self.blackboard["emotion_classes"].append(emo_class)

		emos = []
		for (n,p,mi,mx,di,dx) in zip (names, probs, mins, maxs, dins, daxs):
			emo = Emotion(n)
			emo.probability = p
			emo.min_intensity = mi
			emo.max_intensity = mx
			emo.min_duration = di
			emo.max_duration = dx
			emos.append(emo)

		self.blackboard[emo_class] = emos

	def unpack_config_gestures(self, config, ges_class) :

		def get_values(from_config, num_values):
			rtn_values = [float(z.strip()) for z in from_config.split(",")]
			if len(rtn_values) != num_values:
				raise Exception("List lengths don't match!")
			return rtn_values

		names = [x.strip() for x in config.get("gesture", ges_class).split(",")]
		numb = len(names)

		probs = get_values(config.get("gesture", \
			ges_class + "_probabilities"), numb)
		mins = get_values(config.get("gesture", \
			ges_class + "_intensity_min"), numb)
		maxs = get_values(config.get("gesture", \
			ges_class + "_intensity_max"), numb)

		rins = get_values(config.get("gesture", \
			ges_class + "_repeat_min"), numb)
		raxs = get_values(config.get("gesture", \
			ges_class + "_repeat_max"), numb)

		sins = get_values(config.get("gesture", \
			ges_class + "_speed_min"), numb)
		saxs = get_values(config.get("gesture", \
			ges_class + "_speed_max"), numb)

		self.blackboard["gesture_classes"].append(ges_class)

		gestures = []
		for (n,p,mi,mx,ri,rx,si,sa) in zip (names, probs, mins, maxs, rins, raxs, sins, saxs):
			ges = Gesture(n)
			ges.probability = p
			ges.min_intensity = mi
			ges.max_intensity = mx
			ges.min_repeat = ri
			ges.max_repeat = rx
			ges.min_speed = si
			ges.max_speed = sa
			gestures.append(ges)

		self.blackboard[ges_class] = gestures

	def unpack_config_look_around(self, config):
		def get_values(from_config, num_values):
			rtn_values = [float(z.strip()) for z in from_config.split(",")]
			if len(rtn_values) != num_values:
				raise Exception("List lengths don't match!")
			return rtn_values

		x_coordinates = [float(x.strip()) for x in config.get("boredom", "search_for_attention_x").split(",")]
		numb = len(x_coordinates)

		y_coordinates = get_values(config.get("boredom", "search_for_attention_y"), numb)
		z_coordinates = get_values(config.get("boredom", "search_for_attention_z"), numb)

		for (x, y, z) in zip (x_coordinates, y_coordinates, z_coordinates):
			trg = Target()
			trg.x = x
			trg.y = y
			trg.z = z
			self.blackboard["search_for_attention_targets"].append(trg)


	def __init__(self):

		self.blackboard = blackboard.Blackboard("rig expressions")

		config = ConfigParser.ConfigParser()
		config.readfp(open(os.path.join(os.path.dirname(__file__), "../behavior.cfg")))
		self.blackboard["current_emotion"] = config.get("emotion", "default_emotion")
		self.blackboard["current_emotion_intensity"] = config.getfloat("emotion", "default_emotion_intensity")
		self.blackboard["current_emotion_duration"] = config.getfloat("emotion", "default_emotion_duration")
		self.blackboard["emotion_classes"] = []
		self.blackboard["gesture_classes"] = []
		self.blackboard["emotion_scale_stage"] = config.getfloat("emotion", "emotion_scale_stage")
		self.blackboard["emotion_scale_closeup"] = config.getfloat("emotion", "emotion_scale_closeup")
		self.blackboard["gesture_scale_stage"] = config.getfloat("gesture", "gesture_scale_stage")
		self.blackboard["gesture_scale_closeup"] = config.getfloat("gesture", "gesture_scale_closeup")

		self.unpack_config_emotions(config, "frustrated_emotions")

		self.unpack_config_emotions(config, "positive_emotions")
		self.unpack_config_emotions(config, "non_positive_emotion")

		self.unpack_config_emotions(config, "bored_emotions")
		self.unpack_config_emotions(config, "non_bored_emotion")

		self.unpack_config_emotions(config, "sleep_emotions")
		self.unpack_config_emotions(config, "non_sleep_emotion")

		self.unpack_config_emotions(config, "wake_up_emotions")

		self.unpack_config_emotions(config, "new_arrival_emotions")

		self.unpack_config_gestures(config, "positive_gestures")

		self.unpack_config_gestures(config, "bored_gestures")

		self.unpack_config_gestures(config, "sleep_gestures")

		self.unpack_config_gestures(config, "wake_up_gestures")

		self.blackboard["min_duration_for_interaction"] = config.getfloat("interaction", "duration_min")
		self.blackboard["max_duration_for_interaction"] = config.getfloat("interaction", "duration_max")
		self.blackboard["time_to_change_face_target_min"] = config.getfloat("interaction", "time_to_change_face_target_min")
		self.blackboard["time_to_change_face_target_max"] = config.getfloat("interaction", "time_to_change_face_target_max")
		self.blackboard["glance_probability"] = config.getfloat("interaction", "glance_probability")
		self.blackboard["glance_probability_for_new_faces"] = config.getfloat("interaction", "glance_probability_for_new_faces")
		self.blackboard["glance_probability_for_lost_faces"] = config.getfloat("interaction", "glance_probability_for_lost_faces")
		self.blackboard["z_pitch_eyes"] = config.getfloat("interaction", "z_pitch_eyes")
		self.blackboard["max_glance_distance"] = config.getfloat("interaction", "max_glance_distance")
		self.blackboard["face_study_probabilities"] = config.getfloat("interaction", "face_study_probabilities")
		self.blackboard["face_study_duration_min"] = config.getfloat("interaction", "face_study_duration_min")
		self.blackboard["face_study_duration_max"] = config.getfloat("interaction", "face_study_duration_max")
		self.blackboard["face_study_z_pitch_nose"] = config.getfloat("interaction", "face_study_z_pitch_nose")
		self.blackboard["face_study_z_pitch_mouth"] = config.getfloat("interaction", "face_study_z_pitch_mouth")
		self.blackboard["face_study_y_pitch_left_ear"] = config.getfloat("interaction", "face_study_y_pitch_left_ear")
		self.blackboard["face_study_y_pitch_right_ear"] = config.getfloat("interaction", "face_study_y_pitch_right_ear")
		self.blackboard["sleep_probability"] = config.getfloat("boredom", "sleep_probability")
		self.blackboard["sleep_duration_min"] = config.getfloat("boredom", "sleep_duration_min")
		self.blackboard["sleep_duration_max"] = config.getfloat("boredom", "sleep_duration_max")
		self.blackboard["search_for_attention_index"] = 0
		self.blackboard["search_for_attention_duration_min"] = config.getfloat("boredom", "search_for_attention_duration_min")
		self.blackboard["search_for_attention_duration_max"] = config.getfloat("boredom", "search_for_attention_duration_max")
		self.blackboard["search_for_attention_targets"] = []
		self.unpack_config_look_around(config)
		self.blackboard["wake_up_probability"] = config.getfloat("boredom", "wake_up_probability")
		self.blackboard["time_to_wake_up"] = config.getfloat("boredom", "time_to_wake_up")

		##### Other System Variables #####
		self.blackboard["show_expression_since"] = None

		# ID's of faces newly seen, or lost. Integer ID.
		self.blackboard["new_face"] = 0
		self.blackboard["lost_face"] = 0
		# IDs of faces in the scene, updated once per cycle
		self.blackboard["face_targets"] = []
		# IDs of faces in the scene, updated immediately
		self.blackboard["background_face_targets"] = []
		self.blackboard["current_glance_target"] = 0
		self.blackboard["current_face_target"] = 0
		self.blackboard["interact_with_face_target_since"] = 0.0
		self.blackboard["sleep_since"] = 0.0
		self.blackboard["bored_since"] = 0.0
		self.blackboard["is_interruption"] = False
		self.blackboard["is_sleeping"] = False
		self.blackboard["behavior_tree_on"] = False
		self.blackboard["stage_mode"] = False
		# Flags to indicate which part of the face will be studied
		self.blackboard["face_study_nose"] = False
		self.blackboard["face_study_mouth"] = False
		self.blackboard["face_study_left_ear"] = False
		self.blackboard["face_study_right_ear"] = False


		##### ROS Connections #####
		self.facetrack = FaceTrack(self.blackboard)

		rospy.Subscriber("behavior_switch", String, self.behavior_switch_callback)
		rospy.Subscriber("/blender_api/available_emotion_states",
			AvailableEmotionStates, self.get_emotion_states_cb)

		rospy.Subscriber("/blender_api/available_gestures",
			AvailableGestures, self.get_gestures_cb)

		# Emotional content that the chatbot perceived i.e. did it hear
		# (or reply with) angry words, polite words, etc?
		# currently supplying string rather than specific EmotionState with timing,
		# allowing that to be handled here where timings have been tuned
		print ' setting up chatbot_affect link'
		rospy.Subscriber("/eva/chatbot_affect_perceive", String,
			self.chatbot_affect_perceive_callback)

		self.do_pub_gestures = True
		self.do_pub_emotions = True
		self.emotion_pub = rospy.Publisher("/blender_api/set_emotion_state",
			EmotionState, queue_size=1)
		self.gesture_pub = rospy.Publisher("/blender_api/set_gesture",
			SetGesture, queue_size=1)
		self.affect_pub = rospy.Publisher("/eva/chatbot_affect_express",
			EmotionState, queue_size=1)
		self.tree = self.build_tree()
		time.sleep(0.1)

		log_filename = os.path.join(os.path.abspath(
				os.path.dirname(__file__)), "../bhlog.csv")
		self.log_file = open(log_filename, 'wb')
		self.log_file.write('Action,Timestamp,Event\n')
		try:
			while not rospy.is_shutdown():
				self.tree.next()
		finally:
			self.log_file.close()


	# Pick a random expression out of the class of expressions,
	# and display it. Return the display emotion, or None if none
	# were picked.  Optional argument force guarantees a return.
	def pick_random_expression(self, emo_class_name, trigger, force=False):
		random_number = random.random()
		tot = 0
		emo = None
		emos = self.blackboard[emo_class_name]
		# print emos
		for emotion in emos:
			tot += emotion.probability
			if random_number <= tot:
				emo = emotion
				break
		if emo:
			if force==True:
				intensity = (emo.min_intensity + emo.max_intensity)/2.0
				duration = emo.max_duration
			else:
				intensity = random.uniform(emo.min_intensity, emo.max_intensity)
				duration = random.uniform(emo.min_duration, emo.max_duration)
			self.show_emotion(emo.name, intensity, duration, trigger)
		elif force==True:
			# force said show something but nothing picked, so choose first
			print 'force branch chosen'
			emo=emos[0]
			intensity = 0.6 * emo.max_intensity
			duration = emo.max_duration
			self.show_emotion(emo.name, intensity, duration, trigger)

		return emo

	def pick_random_gesture(self, ges_class_name, trigger):
		random_number = random.random()
		tot = 0
		ges = None
		gestures = self.blackboard[ges_class_name]
		for gesture in gestures:
			tot += gesture.probability
			if random_number <= tot:
				ges = gesture
				break

		if ges:
			intensity = random.uniform(ges.min_intensity, ges.max_intensity)
			repeat = random.uniform(ges.min_repeat, ges.max_repeat)
			speed = random.uniform(ges.min_speed, ges.max_speed)
			self.show_gesture(ges.name, intensity, repeat, speed, trigger)

		return ges


	# Pick the name of a random emotion, excluding those from
	# the exclude list
	def pick_random_emotion_name(self, exclude) :
		ixnay = [ex.name for ex in exclude]
		emos = self.blackboard["emotions"]
		if None == emos:
			return None
		emo_name = random.choice([other for other in emos if other not in ixnay])
		return emo_name

	# Pick a  so-called "instant" or "flash" expression
	def pick_instant(self, emo_class, exclude_class, trigger) :
		emo = self.pick_random_expression(exclude_class)
		if emo :
			exclude = self.blackboard[emo_class]
			emo_name = self.pick_random_emotion_name(exclude)
			tense = random.uniform(emo.min_intensity, emo.max_intensity)
			durat = random.uniform(emo.min_duration, emo.max_duration)
			self.show_emotion(emo_name, tense, durat, trigger)
			print "----- Instant expression: " + emo_name + " (" + \
			     str(tense) + ") for " + str(durat) + " seconds"
		return emo_name

	# ------------------------------------------------------------------
	# The various behavior trees

	# Actions that are taken when a face becomes visible.
	# If there were no people in the scene, she always interacts with that person
	# If she is already interacting with someone else in the scene,
	# she will either glance at the new face or ignore it, depends on the dice roll
	# If she has been interacting with another person for a while,
	# the probability of glancing at a new face is higher
	def someone_arrived(self) :
		tree = owyl.sequence(
			self.is_someone_arrived(),
			owyl.selector(
				##### There previously were no people in the scene #####
				owyl.sequence(
					self.were_no_people_in_the_scene(),
					self.assign_face_target(variable="current_face_target", value="new_face"),
					self.record_start_time(variable="interact_with_face_target_since"),
					self.show_expression(emo_class="new_arrival_emotions", trigger="someone_arrived"),
					self.interact_with_face_target(id="current_face_target", new_face=True, trigger="someone_arrived")
				),

				##### Currently interacting with someone #####
				owyl.sequence(
					self.is_interacting_with_someone(),
					self.dice_roll(event="glance_new_face"),
					self.glance_at_new_face(trigger="someone_arrived")
				),

				##### Does Nothing #####
				owyl.sequence(
					self.print_status(str="----- Ignoring the new face!"),
					self.log(behavior="ignore_face", trigger="someone_arrived"),
					owyl.succeed()
				)
			),
			self.clear_new_face_target()
		)
		return tree

	# ---------------------------
	# Actions that are taken when a face leaves
	# If she was interacting with that person, she will be frustrated
	# If she was interacting with someone else,
	# she will either glance at the lost face or ignore it, depends on the dice roll
	def someone_left(self) :
		tree = owyl.sequence(
			self.is_someone_left(),
			owyl.selector(
				##### Was Interacting With That Person #####
				owyl.sequence(
					self.was_interacting_with_that_person(),
					self.return_to_neutral_position(trigger="someone_left"),
					self.show_frustrated_expression(trigger="someone_left")
				),

				##### Is Interacting With Someone Else #####
				owyl.sequence(
					self.is_interacting_with_someone(),
					self.dice_roll(event="glance_lost_face"),
					self.glance_at_lost_face(trigger="someone_left")
				),

				##### Does Nothing #####
				owyl.sequence(
					self.print_status(str="----- Ignoring the lost face!"),
					self.log(behavior="ignore_face", trigger="someone_left"),
					owyl.succeed()
				)
			),
			self.clear_lost_face_target()
		)
		return tree

	# -----------------------------
	# Interact with people
	# If she is not currently interacting with anyone, or it's time to switch target
	# she will start interacting with someone else
	# Otherwise she will continue with the current interaction
	# she may also glance at other people if there are more than one people in the scene
	def interact_with_people(self) :
		tree = owyl.sequence(
			self.is_face_target(),
			owyl.selector(
				##### Start A New Interaction #####
				owyl.sequence(
					owyl.selector(
						self.is_not_interacting_with_someone(),
						owyl.sequence(
							self.is_more_than_one_face_target(),
							self.is_time_to_change_face_target()
						)
					),
					self.select_a_face_target(),
					self.record_start_time(variable="interact_with_face_target_since"),
					self.interact_with_face_target(id="current_face_target", new_face=False, trigger="people_in_scene")
				),

				##### Glance At Other Faces & Continue With The Last Interaction #####
				owyl.sequence(
					self.print_status(str="----- Continue interaction"),
					owyl.selector(
						owyl.sequence(
							self.is_more_than_one_face_target(),
							self.dice_roll(event="group_interaction"),
							self.select_a_glance_target(),
							self.glance_at(id="current_glance_target", trigger="people_in_scene")
						),
						owyl.succeed()
					),
					self.interact_with_face_target(id="current_face_target", new_face=False, trigger="people_in_scene"),
					owyl.selector(
						owyl.sequence(
							self.dice_roll(event="face_study_saccade"),
							self.face_study_saccade(id="current_face_target", trigger="people_in_scene")
						),
						owyl.succeed()
					)
				)
			)
		)
		return tree


	# -------------------
	# Nothing interesting is happening.
	# She will look around and search for attention.  She may go to sleep,
	# and it's more likely to happen if she has been bored for a while.
	# She wakes up whenever there's an interruption, e.g. someone arrives
	# or after a timeout.
	def nothing_is_happening(self) :
		tree = owyl.sequence(
			owyl.selector(
				##### Is Not Sleeping #####
				owyl.sequence(
					self.is_not_sleeping(),
					owyl.selector(
						##### Go To Sleep #####
						owyl.sequence(
							self.dice_roll(event="go_to_sleep"),
							self.record_start_time(variable="sleep_since"),
							self.print_status(str="----- Go to sleep!"),
							self.go_to_sleep(trigger="nothing_is_happening")
						),

						##### Search For Attention #####
						self.search_for_attention(trigger="nothing_is_happening")
					)
				),

				##### Is Sleeping #####
				owyl.selector(
					##### Wake Up #####
					owyl.sequence(
						self.dice_roll(event="wake_up"),
						self.is_time_to_wake_up(),
						self.wake_up(trigger="time_to_wake_up"),
					),

					##### Continue To Sleep #####
					owyl.sequence(
						self.print_status(str="----- Continue to sleep."),
						self.go_to_sleep(trigger="nothing_is_happening")
					)
				)
			),

			##### If Interruption && Sleeping -> Wake Up #####
			owyl.sequence(
				self.is_interruption(),
				self.is_sleeping(),
				self.wake_up(trigger="interruption"),
				self.print_status(str="----- Interruption: Wake up!"),
			)
		)
		return tree

	# ------------------------------------------------------------------
	# Build the main tree
	def build_tree(self):
		eva_behavior_tree = \
			owyl.repeatAlways(
				owyl.selector(
					owyl.sequence(
						self.is_behavior_tree_on(),
						self.sync_variables(),
						########## Main Events ##########
						owyl.selector(
							self.someone_arrived(),
							self.someone_left(),
							self.interact_with_people(),
							self.nothing_is_happening()
						)
					),
					self.idle_spin()
				)
			)
		return owyl.visit(eva_behavior_tree, blackboard=self.blackboard)

	# Print a single status message
	@owyl.taskmethod
	def print_status(self, **kwargs):
		print kwargs["str"]
		yield True

	# Print emotional state
	@owyl.taskmethod
	def sync_variables(self, **kwargs):
		self.blackboard["face_targets"] = self.blackboard["background_face_targets"]
		yield True

	@owyl.taskmethod
	def dice_roll(self, **kwargs):
		if kwargs["event"] == "glance_new_face":
			if self.blackboard["glance_probability_for_new_faces"] > 0 and self.blackboard["interact_with_face_target_since"] > 0:
				skew = (time.time() - self.blackboard["interact_with_face_target_since"]) / self.blackboard["time_to_change_face_target_max"]
				if random.random() < self.blackboard["glance_probability_for_new_faces"] + skew:
					yield True
				else:
					yield False
			else:
				yield False
		elif kwargs["event"] == "group_interaction":
			if random.random() < self.blackboard["glance_probability"]:
				yield True
			else:
				yield False
		elif kwargs["event"] == "face_study_saccade":
			if random.random() < self.blackboard["face_study_probabilities"]:
				yield True
			else:
				yield False
		elif kwargs["event"] == "go_to_sleep":
			if self.blackboard["sleep_probability"] > 0 and self.blackboard["bored_since"] > 0:
				skew = (time.time() - self.blackboard["bored_since"]) / \
					   (self.blackboard["search_for_attention_duration_max"] / self.blackboard["sleep_probability"])
				if random.random() < self.blackboard["sleep_probability"] + skew:
					yield True
				else:
					yield False
			else:
				yield False
		elif kwargs["event"] == "wake_up":
			if random.random() < self.blackboard["wake_up_probability"]:
				yield True
			else:
				yield False
		else:
			if random.random() > 0.5:
				yield True
			else:
				yield False

	@owyl.taskmethod
	def is_someone_arrived(self, **kwargs):
		self.blackboard["is_interruption"] = False
		if self.blackboard["new_face"] > 0:
			self.blackboard["bored_since"] = 0
			print("----- Someone arrived! id: " + str(self.blackboard["new_face"]))
			yield True
		else:
			yield False

	@owyl.taskmethod
	def is_someone_left(self, **kwargs):
		self.blackboard["is_interruption"] = False
		if self.blackboard["lost_face"] > 0:
			print("----- Someone left! id: " + str(self.blackboard["lost_face"]))
			yield True
		else:
			yield False

	@owyl.taskmethod
	def is_interacting_with_someone(self, **kwargs):
		if self.blackboard["current_face_target"]:
			"----- Is Interacting With Someone!"
			yield True
		else:
			yield False

	@owyl.taskmethod
	def is_not_interacting_with_someone(self, **kwargs):
		if not self.blackboard["current_face_target"]:
			yield True
		else:
			yield False

	@owyl.taskmethod
	def were_no_people_in_the_scene(self, **kwargs):
		if len(self.blackboard["face_targets"]) == 1:
			print("----- Previously, no one in the scene!")
			yield True
		else:
			yield False

	@owyl.taskmethod
	def was_interacting_with_that_person(self, **kwargs):
		if self.blackboard["current_face_target"] == self.blackboard["lost_face"]:
			self.blackboard["current_face_target"] = 0
			print("----- Lost face " + str(self.blackboard["lost_face"]) +
				", but was interacting with them!")
			yield True
		else:
			yield False

	@owyl.taskmethod
	def is_face_target(self, **kwargs):
		if len(self.blackboard["face_targets"]) > 0:
			yield True
		else:
			yield False

	@owyl.taskmethod
	def is_more_than_one_face_target(self, **kwargs):
		if len(self.blackboard["face_targets"]) > 1:
			yield True
		else:
			yield False

	@owyl.taskmethod
	def is_time_to_change_face_target(self, **kwargs):
		if self.blackboard["interact_with_face_target_since"] > 0 and \
				(time.time() - self.blackboard["interact_with_face_target_since"]) >= \
						random.uniform(self.blackboard["time_to_change_face_target_min"], self.blackboard["time_to_change_face_target_max"]):
			print "----- Time to start a new interaction!"
			yield True
		else:
			yield False

	@owyl.taskmethod
	def is_time_to_wake_up(self, **kwargs):
		if self.blackboard["sleep_since"] > 0 and (time.time() - self.blackboard["sleep_since"]) >= self.blackboard["time_to_wake_up"]:
			yield True
		else:
			yield False

	@owyl.taskmethod
	def is_sleeping(self, **kwargs):
		if self.blackboard["is_sleeping"]:
			yield True
		else:
			yield False

	@owyl.taskmethod
	def is_not_sleeping(self, **kwargs):
		if not self.blackboard["is_sleeping"]:
			yield True
		else:
			yield False

	@owyl.taskmethod
	def is_interruption(self, **kwargs):
		if self.blackboard["is_interruption"]:
			yield True
		else:
			yield False

	@owyl.taskmethod
	def is_behavior_tree_on(self, **kwargs):
		if self.blackboard["behavior_tree_on"]:
			yield True
		else:
			yield False

	@owyl.taskmethod
	def assign_face_target(self, **kwargs):
		self.blackboard[kwargs["variable"]] = self.blackboard[kwargs["value"]]
		yield True

	@owyl.taskmethod
	def select_a_face_target(self, **kwargs):
		self.blackboard["current_face_target"] = FaceTrack.random_face_target(self.blackboard["face_targets"])
		yield True

	@owyl.taskmethod
	def select_a_glance_target(self, **kwargs):
		self.blackboard["current_glance_target"] = FaceTrack.random_face_target(self.blackboard["face_targets"], self.blackboard["current_face_target"])
		yield True

	@owyl.taskmethod
	def record_start_time(self, **kwargs):
		self.blackboard[kwargs["variable"]] = time.time()
		yield True

	@owyl.taskmethod
	def interact_with_face_target(self, **kwargs):
		face_id = self.blackboard[kwargs["id"]]
		trigger = kwargs["trigger"]
		self.facetrack.look_at_face(face_id)
		self.write_log("look_at_" + str(face_id), time.time(), trigger)

		if self.should_show_expression("positive_emotions") or kwargs["new_face"]:
			# Show a positive expression, either with or without an instant expression in advance
			if random.random() < self.blackboard["non_positive_emotion_probabilities"]:
				self.pick_instant("positive_emotions", "non_positive_emotion", trigger)
			else:
				self.pick_random_expression("positive_emotions", trigger)

		##### Show A Positive Gesture #####
		self.pick_random_gesture("positive_gestures", trigger)

		interval = 0.01
		duration = random.uniform(self.blackboard["min_duration_for_interaction"], self.blackboard["max_duration_for_interaction"])
		print "----- Interacting w/face id:" + str(face_id) + " for " + str(duration)[:5] + " seconds"
		self.break_if_interruptions(interval, duration)
		yield True

	@owyl.taskmethod
	def face_study_saccade(self, **kwargs):
		face_id = self.blackboard[kwargs["id"]]
		duration = random.uniform(self.blackboard["face_study_duration_min"], self.blackboard["face_study_duration_max"])

		# Randomly pick which part of the face to study
		which_part = random.randint(1, 4)
		if which_part == 1:
			self.blackboard["face_study_nose"] = True
			print "----- Studying face:" + str(face_id) + " (nose)"
		elif which_part == 2:
			self.blackboard["face_study_mouth"] = True
			print "----- Studying face:" + str(face_id) + " (mouth)"
		elif which_part == 3:
			self.blackboard["face_study_left_ear"] = True
			print "----- Studying face:" + str(face_id) + " (left ear)"
		elif which_part == 4:
			self.blackboard["face_study_right_ear"] = True
			print "----- Studying face:" + str(face_id) + " (right ear)"

		self.facetrack.study_face(face_id, duration)
		self.write_log("face_study", time.time(), kwargs["trigger"])
		yield True

	@owyl.taskmethod
	def glance_at(self, **kwargs):
		face_id = self.blackboard[kwargs["id"]]
		print "----- Glancing at face:" + str(face_id)
		glance_seconds = 1
		self.facetrack.glance_at_face(face_id, glance_seconds)
		self.write_log("glance_at_" + str(face_id), time.time(), kwargs["trigger"])
		yield True

	@owyl.taskmethod
	def glance_at_new_face(self, **kwargs):
		face_id = self.blackboard["new_face"]
		print "----- Glancing at new face:" + str(face_id)
		glance_seconds = 1
		self.facetrack.glance_at_face(face_id, glance_seconds)
		self.write_log("glance_at_" + str(face_id), time.time(), kwargs["trigger"])
		yield True

	@owyl.taskmethod
	def glance_at_lost_face(self, **kwargs):
		print "----- Glancing at lost face:" + str(self.blackboard["lost_face"])
		face_id = self.blackboard["lost_face"]
		self.facetrack.glance_at_face(face_id, 1)
		self.write_log("glance_at_" + str(face_id), time.time(), kwargs["trigger"])
		yield True

	@owyl.taskmethod
	def show_expression(self, **kwargs):
		self.pick_random_expression(kwargs["emo_class"], kwargs["trigger"])
		yield True

	@owyl.taskmethod
	def show_frustrated_expression(self, **kwargs):
		self.pick_random_expression("frustrated_emotions", kwargs["trigger"])
		yield True

	@owyl.taskmethod
	def return_to_neutral_position(self, **kwargs):
		self.facetrack.look_at_face(0)
		self.write_log("look_at_neutral", time.time(), kwargs["trigger"])
		yield True

	# Accept an expression name, intensity and duration, and publish it
	# as a ROS message both to blender, and to the chatbot.  Currently,
	# exactly the same message format is used for both blender and the
	# chatbot. This may change in the future(?)
	def show_emotion(self, expression, intensity, duration, trigger):

		# Try to avoid showing more than one expression at once
		now = time.time()
		since = self.blackboard["show_expression_since"]
		durat = self.blackboard["current_emotion_duration"]
		if since is not None and (now - since < 0.7 * durat) :
			return

		# Update the blackboard
		self.blackboard["current_emotion"] = expression
		self.blackboard["current_emotion_intensity"] = intensity
		self.blackboard["current_emotion_duration"] = duration

		# Create the message
		exp = EmotionState()
		exp.name = self.blackboard["current_emotion"]
		exp.magnitude = self.blackboard["current_emotion_intensity"]
		intsecs = int(duration)
		exp.duration.secs = intsecs
		exp.duration.nsecs = 1000000000 * (duration - intsecs)
		# emotion_pub goes to blender and tts;
		if (self.do_pub_emotions) :
			self.emotion_pub.publish(exp)
			self.write_log(exp.name, time.time(), trigger)

		print "----- Show expression: " + expression + " (" + str(intensity)[:5] + ") for " + str(duration)[:4] + " seconds"
		self.blackboard["show_expression_since"] = time.time()

	# Accept an gesture name, intensity, repeat (perform how many times)
	# and speed and then publish it as a ros message.
	def show_gesture(self, gesture, intensity, repeat, speed, trigger):
		ges = SetGesture()
		ges.name = gesture
		ges.magnitude = intensity
		ges.repeat = repeat
		ges.speed = speed
		if (self.do_pub_gestures) :
			self.gesture_pub.publish(ges)
			self.write_log(ges.name, time.time(), trigger)

		print "----- Show gesture: " + gesture + " (" + str(intensity)[:5] + ")"

	@owyl.taskmethod
	def search_for_attention(self, **kwargs):
		print("----- Search for attention")
		trigger = kwargs["trigger"]
		if self.blackboard["bored_since"] == 0:
			self.blackboard["bored_since"] = time.time()

		# Send out the look around msg
		current_idx = self.blackboard["search_for_attention_index"]
		look_around_trg = self.blackboard["search_for_attention_targets"][current_idx]
		self.facetrack.look_pub.publish(look_around_trg)
		self.write_log("look_around", time.time(), trigger)

		# Update / reset the index
		if self.blackboard["search_for_attention_index"] + 1 < len(self.blackboard["search_for_attention_targets"]):
			self.blackboard["search_for_attention_index"] += 1
		else:
			self.blackboard["search_for_attention_index"] = 0

		if self.should_show_expression("bored_emotions"):
			# Show a bored expression, either with or without an instant expression in advance
			if random.random() < self.blackboard["non_bored_emotion_probabilities"]:
				self.pick_instant("bored_emotions", "non_bored_emotion", trigger)
			else:
				self.pick_random_expression("bored_emotions", trigger)

		##### Show A Bored Gesture #####
		self.pick_random_gesture("bored_gestures", trigger)

		interval = 0.01
		duration = random.uniform(self.blackboard["search_for_attention_duration_min"], self.blackboard["search_for_attention_duration_max"])
		self.break_if_interruptions(interval, duration)
		yield True

	# To determine whether it is a good time to show another expression
	# Can be used to avoid making expressions too frequently
	def should_show_expression(self, emo_class):
		since = self.blackboard["show_expression_since"]
		if since is not None and (time.time() - since) >= (self.blackboard["current_emotion_duration"] / 4):
			return True
		else:
			return False

	@owyl.taskmethod
	def go_to_sleep(self, **kwargs):
		self.blackboard["is_sleeping"] = True
		self.blackboard["bored_since"] = 0.0

		##### Show A Sleep Expression #####
		self.pick_random_emotion_name(self.blackboard["sleep_emotions"])

		##### Show A Sleep Gesture #####
		self.pick_random_gesture("sleep_gestures", "nothing_is_happening")

		interval = 0.01
		duration = random.uniform(self.blackboard["sleep_duration_min"], self.blackboard["sleep_duration_max"])
		self.break_if_interruptions(interval, duration)
		yield True

	@owyl.taskmethod
	def wake_up(self, **kwargs):
		print "----- Wake up!"
		trigger = kwargs["trigger"]
		self.blackboard["is_sleeping"] = False
		self.blackboard["sleep_since"] = 0.0
		self.blackboard["bored_since"] = 0.0

		##### Show A Wake Up Expression #####
		self.pick_random_expression("wake_up_emotions", trigger)

		##### Show A Wake Up Gesture #####
		self.pick_random_gesture("wake_up_gestures", trigger)

		yield True

	@owyl.taskmethod
	def clear_new_face_target(self, **kwargs):
		#if not self.blackboard["is_interruption"]:
		print "----- Cleared new face: " + str(self.blackboard["new_face"])
		self.blackboard["new_face"] = 0
		yield True

	@owyl.taskmethod
	def clear_lost_face_target(self, **kwargs):
		print "----- Cleared lost face: " + str(self.blackboard["lost_face"])
		self.blackboard["lost_face"] = 0
		yield True

	# This avoids burning CPU time when the behavior system is off.
	# Mostly it sleeps, and periodically checks for interrpt messages.
	@owyl.taskmethod
	def idle_spin(self, **kwargs):
		if self.blackboard["behavior_tree_on"]:
			yield True

		# Sleep for 1 second.
		time.sleep(1)
		yield True

	def break_if_interruptions(self, interval, duration):
		while duration > 0:
			time.sleep(interval)
			duration -= interval
			if self.blackboard["is_interruption"]:
				break

	# Return the subset of 'core' strings that are in 'avail' strings.
	# Note that 'avail' strings might contain longer names,
	# e.g. "happy-3", whereas core just contains "happy". We want to
	# return "happy-3" in that case, as well as happy-2 and happy-1
	# if they are there.
	def set_intersect(self, emo_class, avail) :
		emos = self.blackboard[emo_class]
		rev = []
		for emo in emos:
			for a in avail:
				if emo.name in a:
					# Copy the emotion, but give it the new name!
					nemo = copy.deepcopy(emo)
					nemo.name = a
					rev.append(nemo)

		# Now, renormalize the probabilities
		tot = 0.0
		for emo  in rev:
			tot += emo.probability
		for emo  in rev:
			emo.probability /= tot

		self.blackboard[emo_class] = rev

	@owyl.taskmethod
	def log(self, **kwargs):
		self.write_log(kwargs["behavior"], time.time(), kwargs["trigger"])

	def write_log(self, behavior, log_time, trigger):
		logger = csv.writer(
			self.log_file, delimiter=",", lineterminator='\n')
		logger.writerow([behavior, log_time, trigger])
		self.log_file.flush()

	# Get the list of available emotions. Update our master list,
	# and cull the various subclasses appropriately.
	def get_emotion_states_cb(self, msg) :
		print("Available Emotion States:" + str(msg.data))
		# Update the complete list of emtions.
		self.blackboard["emotions"] = msg.data

		# Reconcile the other classes
		self.set_intersect("frustrated_emotions", msg.data)
		self.set_intersect("positive_emotions", msg.data)
		self.set_intersect("bored_emotions", msg.data)
		self.set_intersect("sleep_emotions", msg.data)
		self.set_intersect("wake_up_emotions", msg.data)
		self.set_intersect("new_arrival_emotions", msg.data)


	def get_gestures_cb(self, msg) :
		print("Available Gestures:" + str(msg.data))

	# Rescale the intensity of the expressions.
	def rescale_intensity(self, emo_scale, gest_scale) :
		for emo_class in self.blackboard["emotion_classes"]:
			for emo in self.blackboard[emo_class]:
				emo.min_intensity *= emo_scale
				emo.max_intensity *= emo_scale

		for ges_class in self.blackboard["gesture_classes"]:
			for ges in self.blackboard[ges_class]:
				ges.min_intensity *= gest_scale
				ges.max_intensity *= gest_scale

	# Turn behaviors on and off.
	def behavior_switch_callback(self, data):
		if data.data == "btree_on":
			self.do_pub_gestures = True
			self.do_pub_emotions = True
			self.blackboard["is_interruption"] = False

			emo_scale = self.blackboard["emotion_scale_closeup"]
			ges_scale = self.blackboard["gesture_scale_closeup"]

			# If the current mode is stage mode, then tone things down.
			if self.blackboard["stage_mode"]:
				print("----- Switch to close-up mode")
				emo_scale /= self.blackboard["emotion_scale_stage"]
				ges_scale /= self.blackboard["gesture_scale_stage"]

			else:
				print("----- Behavior tree enabled, closeup mode.")

			self.rescale_intensity(emo_scale, ges_scale)
			self.blackboard["stage_mode"] = False
			self.blackboard["behavior_tree_on"] = True

		elif data.data == "btree_on_stage":
			self.do_pub_gestures = True
			self.do_pub_emotions = True
			self.blackboard["is_interruption"] = False

			emo_scale = self.blackboard["emotion_scale_stage"]
			ges_scale = self.blackboard["gesture_scale_stage"]

			# If previously in close-up mode, exaggerate the emotions
			# for the stage settting.
			if self.blackboard["behavior_tree_on"] and not self.blackboard["stage_mode"]:
				print("----- Switch to stage mode")
				emo_scale /= self.blackboard["emotion_scale_closeup"]
				ges_scale /= self.blackboard["gesture_scale_closeup"]
			else:
				print("----- Behavior tree enabled, stage mode.")

			self.rescale_intensity(emo_scale, ges_scale)
			self.blackboard["stage_mode"] = True
			self.blackboard["behavior_tree_on"] = True

		elif data.data == "emotion_off":
			rospy.logwarn("emotion expression disabled)")
			self.do_pub_emotions = False

		elif data.data == "gesture_off":
			self.do_pub_gestures = False

		elif data.data == "btree_off":
			self.blackboard["is_interruption"] = True
			self.blackboard["behavior_tree_on"] = False
			self.blackboard["stage_mode"] = False
			print("---- Behavior tree disabled")

	# The perceived emotional content in the message.
	# emo is of type EmotionState
	def chatbot_affect_perceive_callback(self, emo):

		rospy.loginfo('chatbot perceived emo class ='+emo.data)
		# for now pass through to blender using random positive or non_positive class
		# in future we want more cognitive / behavior
		# pick random emotions may not do anything depending on random number so add force optional arg
		force=True

		if emo.data == 'happy':
			chosen_emo=self.pick_random_expression("positive_emotions",force)
		else:
			chosen_emo=self.pick_random_expression("frustrated_emotions",force)
		# publish this message to cause chatbot to emit response if it's waiting
		#

		exp = EmotionState()
		#  getting from blackboard seems to be inconsistent with expected state
		exp.name = self.blackboard["current_emotion"]
		exp.magnitude = 0.5
		# use zero for duration, tts can compute if needed
		exp.duration.secs = 3.0
		exp.duration.nsecs = 0
		self.affect_pub.publish(exp)
		rospy.loginfo('picked and expressed '+chosen_emo.name)
