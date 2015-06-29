#! /usr/bin/env python

#A python script executed in cogserver to initialize all things

import roslib; roslib.load_manifest('minecraft_bot')
import rospy
from minecraft_bot.msg import map_block_msg,visible_blocks_msg,mobs_msg,playerpos_msg

#Note:must import spacetime before atomspace, 
#or the types class won't include spacetime atom types
from opencog.spacetime import SpaceServer,TimeServer
from perception_module import PerceptionManager,types

ss = SpaceServer(ATOMSPACE)
ts = TimeServer(ATOMSPACE,ss)
pm = PerceptionManager(ATOMSPACE,ss,ts)
ss.setTimeServer(ts)

rospy.init_node('OpenCog_Perception')
rospy.Subscriber('block',map_block_msg,pm.processBlockMessage)
#rospy.Subscriber('visible_blocks_data',visible_blocks_msg,pm.processMapMessage)
#rospy.Subscriber('mobs_data',mobs_msg,pm.processMobMessage)
#rospy.Subscriber('playerpos_data',playerpos_msg,pm.processPlayerPosMessage)

#while not rospy.is_shutdown():
#    rospy.sleep(1.)

