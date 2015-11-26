"""
Created by Bradley Sheneman

Modified from World.py from Spock. Cannot modify the built in World representation needed by Spock,
but can send the same data to our own world manager without unpacking first. This will make data
transfer significantly faster by delegating the work of unpacking individual blocks to a specialized
ROS node. This does break some of the 'modularity', but performance will improve tremendously.
"""

from spockbot.plugins.base import pl_announce
from spockbot.plugins.tools import smpmap
from spockbot.mcp import proto

class SendMapDataCore():
    
    def __init__(self):
        pass


    def reset(self):
        self.__init__()


@pl_announce('SendMapData')
class SendMapDataPlugin:
    
    def __init__(self, ploader, settings):
        
        self.event = ploader.requires('Event')
        self.core = SendMapDataCore()
        ploader.provides('SendMapData', self.core)
        
        packets = (0x01, 0x07, 0x03, 0x21, 0x22, 0x23, 0x26)
        packet_handlers = (
                self.handleNewDimension,
                self.handleNewDimension,
                self.handleTimeUpdate,
                self.handleChunkData,
                self.handleBlockChangeMulti,
                self.handleBlockChange,
                self.handleChunkBulk
                )
        
        for i in range(len(packets)):
            ploader.reg_event_handler(
                    (proto.PLAY_STATE, proto.SERVER_TO_CLIENT, packets[i]),
                    packet_handlers[i]
                    )
            ploader.reg_event_handler('disconnect', self.handleDisconnect)

    
    #Time Update - Update World Time
    def handleTimeUpdate(self, name, packet):
        
        #print "received time update:"
        #print packet
        self.event.emit('ros_time_update', packet.data)

    
    #Join Game/Respawn - New Dimension
    def handleNewDimension(self, name, packet):
        
        #print packet
        self.event.emit('ros_new_dimension', packet.data['dimension'])

    
    #Chunk Data - Update World state
    def handleChunkData(self, name, packet):
        
        self.event.emit('ros_chunk_data', packet.data)

    
    #Multi Block Change - Update multiple blocks
    def handleBlockChangeMulti(self, name, packet):
        
        chunk_x = packet.data['chunk_x']*16
        chunk_z = packet.data['chunk_z']*16
        
        for block in packet.data['blocks']:
            self.event.emit('ros_block_update', {
                'x': block['x'] + chunk_x,
                'y': block['y'],
                'z': block['z'] + chunk_z,
                'data': block['block_data']
                })

	
    #Block Change - Update a single block
    def handleBlockChange(self, name, packet):
        
        data = packet.data
        print packet.data['block_data']
        self.event.emit('ros_block_update', {
            'x': data['location']['x'],
            'y': data['location']['y'],
            'z': data['location']['z'],
            'data': data['block_data']

            })


    #Map Chunk Bulk - Update World state
    def handleChunkBulk(self, name, packet):
        
        self.event.emit('ros_chunk_bulk', packet.data)
    
    
    def handleDisconnect(self, name, data):
        
        self.event.emit('ros_world_reset')
