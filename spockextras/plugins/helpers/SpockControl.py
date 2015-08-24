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

from minecraft_bot.msg import entity_msg, entity_exp_meta, entity_global_meta, entity_mob_meta
from minecraft_bot.msg import entity_movement_meta, entity_object_meta, entity_painting_meta, entity_player_meta 

from minecraft_bot.msg import position_msg



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
        self.msgr = ploader.requires('Messenger')
        self.net = ploader.requires('Net')
        
        # the standard Spock plugin. necessary to track position updates
        # and any other state changes to the client from the Minecraft server
        self.clinfo = ploader.requires('ClientInfo')

        # simply load all of the plugins
        ploader.requires('NewMovement')
        ploader.requires('MineAndPlace')
        ploader.requires('SendMapData')
        ploader.requires('SendEntityData')

        #ploader.reg_event_handler('ros_time_update', self.sendTimeUpdate)
        #ploader.reg_event_handler('ros_new_dimension', self.sendNewDimension)
        ploader.reg_event_handler('ros_chunk_data', self.sendChunkData)
        ploader.reg_event_handler('ros_chunk_bulk', self.sendChunkBulk)
        ploader.reg_event_handler('ros_block_update', self.sendBlockUpdate)
        #ploader.reg_event_handler('ros_world_reset', self.sendWorldReset)
        ploader.reg_event_handler('ros_entity_data', self.sendEntityData)


        #event handlers for all client related events
        #ploader.reg_event_handler('cl_login_success', self.sendClientLogin)
        #ploader.reg_event_handler('cl_join_game', self.sendClientJoinGame)
        #ploader.reg_event_handler('cl_spawn_update', self.sendClientSpawnUpdate)
        #ploader.reg_event_handler('cl_health_update', self.sendClientHealthUpdate)
        ploader.reg_event_handler('ros_position_update', self.sendClientPositionUpdate)
        
        self.core = SpockControlCore()
        ploader.provides('SpockControl', self.core)
        
        self.initSpockControlNode()


    def initSpockControlNode(self):
        
        rospy.init_node('spock_controller')
	print("spock control node initialized")
	
        # subscribe to Spock related data streams from ROS
	rospy.Subscriber('movement_data', movement_msg, self.moveTo, queue_size=1)
	#rospy.Subscriber('mine_block_data', mine_block_msg, self.mineBlock, queue_size=1)
	#rospy.Subscriber('place_block_data', place_block_msg, self.placeBlock, queue_size=1)
        #rospy.Subscriber('look_test', position_msg, self.lookTest, queue_size = 1)

        #self.pub_time =      rospy.Publisher('time_data', time_msg, queue_size = 1)
        #self.pub_dim =       rospy.Publisher('dimension_data', dim_msg, queue_size = 1)
        self.pub_chunk =    rospy.Publisher('chunk_data', chunk_data_msg, queue_size = 1000)
        self.pub_block =    rospy.Publisher('block_data', block_data_msg, queue_size = 1000)
        self.pub_bulk =     rospy.Publisher('chunk_bulk', chunk_bulk_msg, queue_size = 1000)
        #self.pub_wstate =    rospy.Publisher('world_state', world_state_msg, queue_size = 1)
        self.pub_entity =   rospy.Publisher('entity_data', entity_msg, queue_size = 100)

        #self.pub_clinfo_login = rospy.Publisher('client_login_data', position_msg, queue_size = 10)
        #self.pub_clinfo_join = rospy.Publisher('client_join_data', position_msg, queue_size = 10)
        #self.pub_clinfo_spawn = rospy.Publisher('client_spawn_data', position_msg, queue_size = 10)
        #self.pub_clinfo_health = rospy.Publisher('client_health_data', position_msg, queue_size = 10)
        self.pub_clinfo_pos = rospy.Publisher('client_position_data', position_msg, queue_size = 100)
    
    def lookTest(self, data):
    
        # test to see what happens when we request pitch and yaw change from Minecraft server
        packet = {}
        packet['x'] = data.x
        packet['y'] = data.y
        packet['z'] = data.z
        #self.clinfo.position.yaw = data.yaw
        packet['on_ground'] = True
        packet['yaw'] = data.yaw
        packet['pitch'] = data.pitch
        
        #everything is relative, for simplicity
        #packet['flags'] = 0b11111
        
        print "sending test packet for look only!"
        print packet
    
        self.net.push_packet('PLAY>Player Position and Look', packet)
        #self.net.push_packet('PLAY>Player Position', packet)




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
    
    #def sendTimeUpdate(self, name, data):
    #    
    #    msg = time_msg()
    #
    #    pub_time.publish(data)
    
    
    #def sendNewDimension(self, data):
    #    
    #    pub_dim.publish(data)
    

    def sendChunkData(self, name, data):
        
        msg = chunk_data_msg()
        self.msgr.setMessage(msg, data)
        
        rospy.loginfo("published chunk message: loc: %d, %d", msg.chunk_x, msg.chunk_z)
        self.pub_chunk.publish(msg)
    
    
    def sendChunkBulk(self, name, data):
        
        msg = chunk_bulk_msg()
        
        self.msgr.setMessage(msg, data)

        meta = []
        for i in range(len(data['metadata'])):
            meta.append(chunk_meta_msg())
            self.msgr.setMessage(meta[i], data['metadata'][i])
        
        msg.metadata = meta
        rospy.loginfo("published chunk bulk message, sky: %s, rostime: %s, mctime: %s", msg.sky_light, msg.ROStimestamp, msg.MCtimestamp)
        
        self.pub_bulk.publish(msg)
    
    
    def sendBlockUpdate(self, name, data):
        
        msg = block_data_msg()
        self.msgr.setMessage(msg, data)
        
        rospy.loginfo("published block update: data, %d loc: %d, %d, %d", 
                msg.data, msg.x, msg.y, msg.z)
        self.pub_block.publish(msg)
    
    
    
    #def sendWorldReset(self, name, data):
    #    
    #    pub_wstate.publish(data)


    def sendEntityData(self, name, data):
        
        print(data)
        msg = entity_msg()
        self.msgr.setMessage(msg, data)

        rospy.loginfo("published entity message: type: %d, uid: %d, loc: %d, %d, %d",
                msg.type, msg.eid, msg.x, msg.y, msg.z)
        self.pub_entity.publish(msg)


    def sendClientLogin(self, name, data):
        
        print "received client login"
        print data


    def sendClientJoinGame(self, name, data):

        print "received client join game"
        print data

    def sendClientSpawnUpdate(self, name, data):
        
        print "received client spawn"
        print data


    def sendClientHealthUpdate(self, name, data):
        
        print "received client health update"
        print data


    def sendClientPositionUpdate(self, name, data):
        
        #print "received client position update"
        #print data
        
        msg = position_msg()
        self.msgr.setMessage(msg, data)
        
        self.pub_clinfo_pos.publish(msg)

