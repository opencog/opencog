#! /usr/bin/env python

#A python script executed in cogserver to initialize all things
import os
import roslib; roslib.load_manifest('minecraft_bot')
import rospy
from opencog.spacetime import SpaceTimeAndAtomSpace
from opencog.atomspace import AtomSpace, types
from opencog.type_constructors import set_type_ctor_atomspace
from opencog.utilities import initialize_opencog, finalize_opencog
from perception_module import PerceptionManager
from attention_module import AttentionController
from action_gen import ActionGenerator
#import actions #import schema

rospy.init_node('OpenCog_Perception')
spacetime = SpaceTimeAndAtomSpace()
full_path = os.path.realpath(__file__)
config_file_name = os.path.dirname(full_path) + "/opencog_python_eval.conf"
print config_file_name
# Initialize Python
initialize_opencog(spacetime.get_atomspace(), config_file_name)

set_type_ctor_atomspace(spacetime.get_atomspace())
pm = PerceptionManager(spacetime.get_atomspace(),
                       spacetime.get_space_server(),
                       spacetime.get_time_server())
ag = ActionGenerator(spacetime.get_atomspace(),
                     spacetime.get_space_server(),
                     spacetime.get_time_server())
ac = AttentionController(spacetime.get_atomspace())

while not rospy.is_shutdown():
    pm.update_perception()
    ac.control_av_in_atomspace()
    ag.generate_action()
    rospy.sleep(2)

