"""
the central ROS node that controls Spock. Will register all events from ROS and transfer them over to
its corresponding spock plugin

all ROS related Spock events are prefaced with 'ros_'
the subscriber callbacks should be named by whatever is after the '_' in camelCase

currently the ROS message is passed along unmodified. this may need to change later

"""

import roslib; roslib.load_manifest('minecraft_bot')
import rospy
from minecraft_bot.msg import movement_msg, place_block_msg, mine_block_msg
from minecraft_bot.msg import chunk_data_msg, chunk_bulk_msg, chunk_meta_msg, block_data_msg


from spock.mcp import mcdata
from spock.utils import pl_announce

import logging
logger = logging.getLogger('spock')


class SpockControlCore:

    def __init__(self):

        self.target = None



@pl_announce('SpockControl')
class SpockControlPlugin:
    
    def __init__(self, ploader, settings):
        
        self.event = ploader.requires('Event')
        
        # simply load all of the plugins
        #ploader.requires('NewMovement')
        #ploader.requires('MineAndPlace')
        ploader.requires('SendMapData')

        #ploader.reg_event_handler('ros_time_update', self.sendTimeUpdate)
        #ploader.reg_event_handler('ros_new_dimension', self.sendNewDimension)
        #ploader.reg_event_handler('ros_chunk_data', self.sendChunkData)
        ploader.reg_event_handler('ros_chunk_bulk', self.sendChunkBulk)
        #ploader.reg_event_handler('ros_block_update', self.sendBlockUpdate)
        #ploader.reg_event_handler('ros_world_reset', self.sendWorldReset)

        self.scc = SpockControlCore()
        ploader.provides('SpockControl', self.scc)
        
        self.initSpockControlNode()


    def initSpockControlNode(self):
        
        rospy.init_node('spock_controller')
	print("spock control node initialized")
	
        # subscribe to Spock related data streams from ROS
	#rospy.Subscriber('movement_data', movement_msg, self.moveTo, queue_size=1)
	#rospy.Subscriber('mine_block_data', mine_block_msg, self.mineBlock, queue_size=1)
	#rospy.Subscriber('place_block_data', place_block_msg, self.placeBlock, queue_size=1)

        #self.pub_time =      rospy.Publisher('time_data', time_msg, queue_size = 1)
        #self.pub_dim =       rospy.Publisher('dimension_data', dim_msg, queue_size = 1)
        self.pub_chunk =     rospy.Publisher('chunk_data', chunk_data_msg, queue_size = 1000)
        self.pub_block =     rospy.Publisher('block_data', block_data_msg, queue_size = 1000)
        self.pub_bulk =      rospy.Publisher('chunk_bulk', chunk_bulk_msg, queue_size = 1000)
        #pub_wstate =    rospy.Publisher('world_state', world_state_msg, queue_size = 1)

    ### ROS Subscriber callbacks simply pass data along to the Spock event handlers
    def moveTo(self, data):

        self.event.emit('ros_moveto', data)
        print("emitting event ros_moveto")

    
    def mineBlock(self, data):

        self.event.emit('ros_mineblock', data)
        print("emitting event ros_mineblock")
    
    
    def placeBlock(self, data):

        self.event.emit('ros_placeblock', data)
        print("emitting event ros_placeblock")


    
    ### Perception handlers. Invoke ROS Publishers
    """
    def sendTimeUpdate(self, name, data):
        
        msg = time_msg()

        pub_time.publish(data)
    
    
    def sendNewDimension(self, data):
        
        pub_dim.publish(data)
    
    """

    def sendChunkData(self, name, data):
        
        msg = chunk_data_msg()
        msg.chunk_x = data['chunk_x']
        msg.chunk_z = data['chunk_z']
        msg.continuous = data['continuous']
        msg.primary_bitmap = data['primary_bitmap']
        msg.data = data['data']
        #print "sent chunk data message"
        #print msg
        self.pub_chunk.publish(msg)
    
    
    def sendChunkBulk(self, name, data):
        
        msg = chunk_bulk_msg()
        
        meta = []
        for i in range(len(data['metadata'])):
            meta.append(chunk_meta_msg())
            meta[i].chunk_x = data['metadata'][i]['chunk_x']
            meta[i].chunk_z = data['metadata'][i]['chunk_z']
            # bulk chunk packet doesn't have continuous field?
	    #meta[i].continuous = data['metadata'][i]['continuous']
            meta[i].primary_bitmap = data['metadata'][i]['primary_bitmap']

        msg.sky_light = data['sky_light']
	msg.metadata = meta
        msg.data = data['data']

        #print "sent chunk bulk message"
        #print msg
        self.pub_bulk.publish(msg)
    
    
    def sendBlockUpdate(self, name, data):
        
        msg = block_data_msg()
        msg.blockid = data['block_data']
        msg.x = data['location']['x']
        msg.y = data['location']['y']
        msg.z = data['location']['z']
        #print "sent block data message"
        #print msg
        self.pub_block.publish(msg)
    
    
    """
    def sendWorldReset(self, name, data):
        
        pub_wstate.publish(data)
    """
