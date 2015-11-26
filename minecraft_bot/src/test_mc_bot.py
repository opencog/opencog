#!/usr/bin/env python

# import roslib; roslib.load_manifest('minecraft_bot')
# import rospy
# from minecraft_bot.msg import movement_msg

from spockbot import Client
from spockbot.plugins import default_plugins
from spockbot.plugins.loader import PluginLoader
from spockbot.plugins.helpers.entities import EntitiesPlugin

# load custom plugins. I use capitalized names to indicate non-standard plugins
from spockextras.plugins.helpers.MineAndPlace import MineAndPlacePlugin
from spockextras.plugins.helpers.NewMovement import NewMovementPlugin
from spockextras.plugins.helpers.NewPhysics import NewPhysicsPlugin
from spockextras.plugins.helpers.SpockControl import SpockControlPlugin

from spockextras.plugins.helpers.Messenger import MessengerPlugin
from spockextras.plugins.helpers.SendMapData import SendMapDataPlugin
from spockextras.plugins.helpers.SendEntityData import SendEntityDataPlugin



# connect to localhost server
settings = {'start': {'username': 'Bot',},'auth': {'authenticated': False,},}
plugins = default_plugins

plugins.append(('Messenger', MessengerPlugin))
plugins.append(('SendMapData', SendMapDataPlugin))
plugins.append(('SendEntityData', SendEntityDataPlugin))


plugins.append(('MineAndPlace', MineAndPlacePlugin))
plugins.append(('NewMovement', NewMovementPlugin))
#plugins.append(('NewPhysics', NewPhysicsPlugin))
plugins.append(('SpockControl', SpockControlPlugin))
client = Client(plugins = plugins, settings = settings)

print("connecting to localhost on port 25565")

#client.start() with no arguments will automatically connect to localhost
client.start('localhost', 25565)

