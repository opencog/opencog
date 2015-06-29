"""
created by Bradley Sheneman

plugin to initialize the world and send data on to ROS. Makes use of the WorldPlugin provided by Spock.
also deals with new chunks becoming visible as the player moves around the world, and thus responds
to player position update events.

"""

from spock.utils import pl_announce
from spock.mcmap import mapdata
from spock.utils import Vec3

import logging
logger = logging.getLogger('spock')

class MapInitCore:

    def __init__(self):
    
    def move(self, direction, motion, jump):

        acc = motions[motion]


@pl_announce('MapInit')
class MapInitPlugin:
    
    def __init__(self, ploader, settings):
        
        self.world = ploader.requires('World')
        self.event = ploader.requires('Event')
        clinfo = ploader.requires('ClientInfo')
        
        ploader.reg_event_handler('world_tick', self.tick)
        #time should probably be handled by a separate plugin
        #ploader.reg_event_handler('w_time_update', handleTimeUpdate)
        ploader.reg_event_handler('w_new_dimension', handleNewDimension)

        self.core = MapInitCore()
        ploader.provides('MapInit', self.core)


    def startMapInit(self):
        
        self.event.emit('ros_world_init_started')
    
    
    def finishMapInit(self):
        
        self.event.emit('ros_world_init_finished')
    

    # basically whenever the player joins or respawns
    def handleNewDimension(self):
        
        startMapInit()
        
        # each column is addressed by a tuple (x, z)
        # this will simply pass the entire structure for the chunk column
        # along with its coordinates
        cols = world.columns
        for col in cols.keys():
            for chunk in col.chunks:
            data = {}
            data['x'] = col[0]
            data['z'] = col[1]
            data['biome'] = cols[col].biome.get(col)
            data['raw'] = world.columns[col]
            self.event.emit('ros_column_update', data)
        
        finishMapInit()

