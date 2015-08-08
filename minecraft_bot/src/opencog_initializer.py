#! /usr/bin/env python

#A python script executed in cogserver to initialize all things

import roslib; roslib.load_manifest('minecraft_bot')
import rospy
from minecraft_bot.msg import map_block_msg,visible_blocks_msg,mobs_msg,playerpos_msg
from minecraft_bot.srv import visible_blocks_srv
from opencog.spacetime import SpaceServer,TimeServer
from perception_module import PerceptionManager,types



ss = SpaceServer(ATOMSPACE)
ts = TimeServer(ATOMSPACE,ss)
pm = PerceptionManager(ATOMSPACE,ss,ts)
ss.set_time_server(ts)

rospy.init_node('OpenCog_Perception')
rospy.wait_for_service('get_visible_blocks')

try:
    getVisBlocksFromSrv = rospy.ServiceProxy('get_visible_blocks', visible_blocks_srv)
except rospy.ServiceException, e:
    print "service call failed: %s" % e

#rospy.Subscriber('block',map_block_msg,pm.processBlockMessage)
#rospy.Subscriber('visible_blocks_data',visible_blocks_msg,pm.processMapMessage)
#rospy.Subscriber('mobs_data',mobs_msg,pm.processMobMessage)
#rospy.Subscriber('playerpos_data',playerpos_msg,pm.processPlayerPosMessage)

#while not rospy.is_shutdown():
#    rospy.sleep(1.)

