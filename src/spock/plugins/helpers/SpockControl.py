"""
the central ROS node that controls Spock. Will register all events from ROS and transfer them over to
its corresponding spock plugin

all ROS related Spock events are prefaced with 'ros_'
the subscriber callbacks should be named by whatever is after the '_' in camelCase

currently the ROS message is passed along unmodified. this may need to change later

"""

import roslib; roslib.load_manifest('minecraft_bot')
import rospy
#from minecraft_bot.msg import movement_msg, place_block_msg, mine_block_msg
from minecraft_bot.msg import visible_blocks_msg,mobs_msg,playerpos_msg


from spock.mcp import mcdata
from spock.utils import pl_announce

import logging
logger = logging.getLogger('spock')

class ROSMobEntity:
        eid=0
        mobtype=0
        x=0
        y=0
        z=0
        head_yaw=0
        head_pitch=0
        velocity_x=0
        velocity_y=0
        velocity_z=0

class SpockControlCore:

    def __init__(self):

        self.target = None



@pl_announce('SpockControl')
class SpockControlPlugin:
    
    def __init__(self, ploader, settings):
        
        self.event = ploader.requires('Event')
        
        # simply load all of the plugins
#        ploader.requires('NewMovement')
#        ploader.requires('MineAndPlace')

        self.scc = SpockControlCore()
        self.world = ploader.requires('World')
        self.player = ploader.requires('ClientInfo')
        ploader.provides('SpockControl', self.scc)
        
        ploader.reg_event_handler('ros_send_visible_blocks', self.sendVisibleBlocks)        
        ploader.reg_event_handler('ros_send_mobs', self.sendMobs)
        ploader.reg_event_handler('cl_position_update', self.sendPlayerPos)

        self.initSpockControlNode()


    def initSpockControlNode(self):
        
        rospy.init_node('spock_controller')
	print("spock control node initialized")
	
        # subscribe to Spock related data streams from ROS
#	rospy.Subscriber('movement_data', movement_msg, self.moveTo, queue_size=1)
#	rospy.Subscriber('mine_block_data', mine_block_msg, self.mineBlock, queue_size=1)
#	rospy.Subscriber('place_block_data', place_block_msg, self.placeBlock, queue_size=1)

        # publish to other ROS node

        self.vis_pub=rospy.Publisher('visible_blocks_data',visible_blocks_msg,queue_size=1)
        self.mob_pub=rospy.Publisher('mobs_data',mobs_msg,queue_size=1)
        self.playerpos_pub=rospy.Publisher('playerpos_data',playerpos_msg,queue_size=1)
    """

    # ROS Subscriber callbacks simply pass data along to the Spock event handlers
    def moveTo(self, data):

        self.event.emit('ros_moveto', data)
        print("emitting event ros_moveto")

    
    def mineBlock(self, data):

        self.event.emit('ros_mineblock', data)
        print("emitting event ros_mineblock")
    
    
    def placeBlock(self, data):

        self.event.emit('ros_placeblock', data)
        print("emitting event ros_placeblock")
    
    """

    #event handler to publish ROS message

    def sendVisibleBlocks(self, packet, data):
        msg=visible_blocks_msg()
        #in Minecraft y direction is up/down, but in Opencog SpaceMap the up/down direction is z direction.
        msg.blocks=map(swapYAndZ,data)
        msg.timestamp=self.world.age
        self.vis_pub.publish(msg)

    def sendMobs(self,packet,data):
        msg=mobs_msg()
        data={ eid:swapYAndZ(mob) for eid,mob in data.items()}
        msg.mobs=[ data[eid] for eid in data]
        msg.timestamp=self.world.age
        self.mob_pub.publish(msg)

    def sendPlayerPos(self,packet,data):
        msg=playerpos_msg()
        swapYAndZ(data)
        msg.x=data.x
        msg.y=data.y
        msg.z=data.z
        msg.yaw=data.yaw
        msg.pitch=data.pitch
        msg.timestamp=self.world.age
        self.playerpos_pub.publish(msg)
                        
    def swapYAndZ(self,data):

        ##in Minecraft y direction is up/down, but in Opencog SpaceMap the up/down direction is z direction.This function is a helper for swapping y and z direction.
        data.y,data.z=data.z,data.y
        return data
