#! /usr/bin/env python

import roslib; roslib.load_manifest('minecraft_bot')
import rospy
from minecraft_bot.msg import controller_msg

from spock import Client, PluginLoader
from spock.plugins import DefaultPlugins
from spock.plugins.helpers.move import MovementPlugin
from spock.plugins.helpers.entities import EntityPlugin
from spock.plugins.helpers.clientinfo import ClientInfoPlugin

# my plugin. I interpret the capitalized name to mean a non-standard plugin
from spockextras.plugins.Runaway import RunAwayPlugin

# connect to localhost server
settings = {'start': {'username': 'Bot',},'auth': {'authenticated': False,},}
plugins = DefaultPlugins

plugins.append(('ClientInfo', ClientInfoPlugin))
plugins.append(('RunAway', RunAwayPlugin))
client = Client(plugins = plugins, settings = settings)

#client.start() with no arguments will automatically connect to localhost
client.start('localhost', 25565)
