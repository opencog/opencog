#! /usr/bin/env python

#A python script executed in cogserver to initialize all things
import roslib; roslib.load_manifest('minecraft_bot')
import rospy
from minecraft_bot.msg import map_block_msg#, visible_blocks_msg, mobs_msg,playerpos_msg
from minecraft_bot.srv import visible_blocks_srv, look_srv, rel_move_srv
from opencog.spacetime import SpaceServer, TimeServer
from opencog.atomspace import AtomSpace, Handle, types
from opencog.type_constructors import *
from opencog.bindlink import execute_atom
from opencog.atomspace import Atom
#from opencog.utilities import initialize_opencog, finalize_opencog
from perception_module import PerceptionManager
from action_gen import ActionGenerator

rospy.init_node('OpenCog_Perception')
atomspace = AtomSpace()
#initialize_opencog(atomspace)
set_type_ctor_atomspace(atomspace)
ss = SpaceServer(atomspace)
ts = TimeServer(atomspace,ss)
ss.set_time_server(ts)
pm = PerceptionManager(atomspace, ss, ts)
#ag = ActionGenerator(atomspace, ss, ts)

while not rospy.is_shutdown():
    pm.update_perception()
 #   ag.generate_action()


