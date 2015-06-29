#! /usr/bin/env python



import sys

from spock import Client, PluginLoader
from spock.plugins import DefaultPlugins
from spock.plugins.helpers.move import MovementPlugin
from spock.plugins.helpers.physics import PhysicsPlugin
from spock.plugins.helpers.entities import EntityPlugin
from spock.plugins.helpers.clientinfo import ClientInfoPlugin
from spock.plugins.helpers.visibility import VisibilityPlugin
from spock.plugins.helpers.SpockControl import SpockControlPlugin

from spockbot.plugins.CursesCommand import *
from spockbot.plugins.BaseCommands import BaseCommandsPlugin
from spockbot.plugins.ChatCommand import ChatCommandPlugin
from spockbot.plugins.Chat import ChatPlugin


import roslib; roslib.load_manifest('minecraft_bot')
import rospy

# connect to localhost server
settings = {'start': {'username': 'Bot',},'auth': {'authenticated': False,},}

plugins = DefaultPlugins
plugins.append(('basecommand', BaseCommandsPlugin))
plugins.append(('chat', ChatPlugin))
plugins.append(('chatcommand', ChatCommandPlugin))
plugins.append(('Movement', MovementPlugin))
plugins.append(('Physics', PhysicsPlugin))
plugins.append(('Entities', EntityPlugin))
plugins.append(('ClientInfo', ClientInfoPlugin))
plugins.append(('Visibility', VisibilityPlugin))
plugins.append(('SpockControl',SpockControlPlugin))
client = Client(plugins = plugins, settings = settings)

#client.start() with no arguments will automatically connect to localhost
client.start('localhost', 25565)
