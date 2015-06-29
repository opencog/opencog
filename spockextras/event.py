"""
Provides the core event loop
"""

import roslib; roslib.load_manifest('minecraft_bot')
import rospy
from minecraft_bot.msg import controller_msg

import sys

import signal
import copy
from spock.mcp import mcdata
from spock.utils import pl_announce

import logging
logger = logging.getLogger('spock')

class EventCore:
	def __init__(self):
		self.kill_event = False
		self.event_handlers = {}
		signal.signal(signal.SIGINT, self.kill)
		signal.signal(signal.SIGTERM, self.kill)

	def event_loop(self):
		rospy.init_node('spock_listener')
		print("spock_listener node initialized")
		# subscribe to random action generator stream
		rospy.Subscriber('controller_data', controller_msg, self.cmd_callback, queue_size=1)

		while not self.kill_event:
			self.emit('event_tick')
		logger.info('Event Kill called, shutting down')
		self.emit('kill')

	def reg_event_handler(self, event, handler):
		if event not in self.event_handlers:
			self.event_handlers[event] = []
		self.event_handlers[event].append(handler)

	def emit(self, event, data = None):
		if event not in self.event_handlers:
			self.event_handlers[event] = []
		to_remove = []
		for handler in self.event_handlers[event]:
			if handler(
				event,
				data.clone() if hasattr(data, 'clone') else copy.deepcopy(data)
			):
				to_remove.append(handler)
		for handler in to_remove:
			self.event_handlers[event].remove(handler)
	
	def cmd_callback(self, data):
		command = int(data.action)
		command_out = ""
		print("received value: " + str(command))
		if command == 1:
			command_out = "cmd_animation"
			print("sent command to spock: " + command_out)
			self.emit(command_out)

	def kill(self, *args):
		self.kill_event = True

@pl_announce('Event')
class EventPlugin:
	def __init__(self, ploader, settings):
		ploader.provides('Event', EventCore())
