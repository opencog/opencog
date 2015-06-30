#!/usr/bin/env python

# to test map data and timer plugins
# import roslib; roslib.load_manifest('minecraft_bot')
# import rospy
# from minecraft_bot.msg import movement_msg

from spock import Client, PluginLoader
from spock.plugins import DefaultPlugins
#from spock.plugins.helpers.entities import EntityPlugin

# load custom plugins. I use capitalized names to indicate non-standard plugins
#from spockextras.plugins.MineAndPlace import MineAndPlacePlugin
#from spockextras.plugins.NewMovement import NewMovementPlugin
#from spockextras.plugins.NewPhysics import NewPhysicsPlugin
<<<<<<< HEAD
from spockextras.plugins.SpockControl import SpockControlPlugin
#from spockextras.plugins.SendMapData import SendMapDataPlugin
from spockextras.plugins.SendEntityData import SendEntityDataPlugin
=======
from spockextras.plugins.helpers.SpockControl import SpockControlPlugin
from spockextras.plugins.helpers.SendMapData import SendMapDataPlugin
>>>>>>> 528e9ef933f98f0fbdf80ed2664e0513c92c5db2

# connect to localhost server
settings = {'start': {'username': 'Bot',},'auth': {'authenticated': False,},}
plugins = DefaultPlugins

#plugins.append(('MineAndPlace', MineAndPlacePlugin))
#plugins.append(('NewMovement', NewMovementPlugin))
#plugins.append(('NewPhysics', NewPhysicsPlugin))
plugins.append(('SpockControl', SpockControlPlugin))
<<<<<<< HEAD
#plugins.append(('SendMapData', SendMapDataPlugin))
plugins.append(('SendEntityData', SendEntityDataPlugin))
=======
plugins.append(('SendMapData', SendMapDataPlugin))
>>>>>>> 528e9ef933f98f0fbdf80ed2664e0513c92c5db2

client = Client(plugins = plugins, settings = settings)

print("connecting to localhost on port 25565")

#client.start() with no arguments will automatically connect to localhost
client.start('localhost', 25565)

