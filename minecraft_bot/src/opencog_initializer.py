#! /usr/bin/env python

#A python script executed in cogserver to initialize all things
import roslib; roslib.load_manifest('minecraft_bot')
import rospy
from opencog.spacetime import SpaceTimeAndAtomSpace
from opencog.atomspace import AtomSpace, types
from opencog.type_constructors import set_type_ctor_atomspace
#from opencog.utilities import initialize_opencog, finalize_opencog
from perception_module import PerceptionManager
from action_gen import ActionGenerator
from actions import * #import schema

rospy.init_node('OpenCog_Perception')
spacetime = SpaceTimeAndAtomSpace()
#initialize_opencog(atomspace)
set_type_ctor_atomspace(spacetime.get_atomspace())
pm = PerceptionManager(spacetime.get_atomspace(),
                       spacetime.get_space_server(),
                       spacetime.get_time_server())
ag = ActionGenerator(spacetime.get_atomspace(),
                     spacetime.get_space_server(),
                     spacetime.get_time_server())

while not rospy.is_shutdown():
    pm.update_perception()
    ag.generate_action()


