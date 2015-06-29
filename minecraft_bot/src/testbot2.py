#!/usr/bin/env python

# import roslib; roslib.load_manifest('minecraft_bot')
# import rospy
# from minecraft_bot.msg import movement_msg

from spock import Client, PluginLoader
from spock.plugins import DefaultPlugins
from spock.plugins.helpers.entities import EntityPlugin

# load custom plugins. I use capitalized names to indicate non-standard plugins
from spockextras.plugins.MineAndPlace import MineAndPlacePlugin
from spockextras.plugins.NewMovement import NewMovementPlugin
from spockextras.plugins.NewPhysics import NewPhysicsPlugin
from spockextras.plugins.SpockControl import SpockControlPlugin


# connect to localhost server
settings = {'start': {'username': 'Bot',},'auth': {'authenticated': False,},}
plugins = DefaultPlugins

plugins.append(('MineAndPlace', MineAndPlacePlugin))
plugins.append(('NewMovement', NewMovementPlugin))
plugins.append(('NewPhysics', NewPhysicsPlugin))
plugins.append(('SpockControl', SpockControlPlugin))

client = Client(plugins = plugins, settings = settings)

print("connecting to localhost on port 25565")

#client.start() with no arguments will automatically connect to localhost
client.start('localhost', 25565)

