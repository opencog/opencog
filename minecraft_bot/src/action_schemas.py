"""
action schemas called by Grounded Predicate/Schema Node
"""

import math
import roslib; roslib.load_manifest('minecraft_bot')
import rospy
from minecraft_bot.srv import look_srv, rel_move_srv, abs_move_srv
from opencog.spacetime import SpaceTimeAndAtomSpace
from opencog.spatial import get_near_free_point
from opencog.atomspace import types, TruthValue
from opencog.type_constructors import *

rospy.wait_for_service('set_relative_look')
rospy.wait_for_service('set_look')
rospy.wait_for_service('set_relative_move')
rospy.wait_for_service('set_move')

atomspace = SpaceTimeAndAtomSpace().get_atomspace()
space_server = SpaceTimeAndAtomSpace().get_space_server()

try:
    _ros_set_relative_look = rospy.ServiceProxy('set_relative_look', look_srv)
    _ros_set_look = rospy.ServiceProxy('set_look', look_srv)
    _ros_set_relative_move = rospy.ServiceProxy('set_relative_move', rel_move_srv)
    _ros_set_move = rospy.ServiceProxy('set_move', abs_move_srv)
except rospy.ServiceException, e:
    print "service call failed: %s"% e

def is_attractive(atom):
    print 'is_attractive'
    sti = atom.av['sti']
    if sti > 1:
        print 'attractive!'
        return TruthValue(1,1)
    else:
        print 'boring'
        return TruthValue(0,1)

def move_toward_block(block_atom):
    print 'move toward atom', block_atom
    jump = False
    map_handle = (atomspace.get_atoms_by_name(
        types.SpaceMapNode, "MCmap")[0]).h
    cur_map = space_server.get_map(map_handle)
    block_pos = cur_map.get_block_location(block_atom.h)
    dest = get_near_free_point(cur_map, block_pos, 2, (1,0,0), True)
    if dest == None:
        print 'get_no_free_point'
    print 'block_pos, dest', block_pos, dest
    self_handle = cur_map.get_self_agent_entity()
    self_pos = cur_map.get_last_appeared_location(self_handle)
    if (math.floor(self_pos[0]) == dest[0]
        and math.floor(self_pos[1]) == dest[1]
        and math.floor(self_pos[2]) == dest[2]):
        print 'has arrived there'
        return TruthValue(1,1)
    else:
        #TODO: In Minecraft the up/down direction is y coord
        # but we should swap y and z in ros node, not here..
        response = _ros_set_move(block_pos[0], block_pos[1], jump)
        rospy.sleep(1)
        print 'action_schemas: abs_move response', response
        if response.state:
            print 'move success'
            return TruthValue(1,1)
        else:
            print 'move fail'
            return TruthValue(0,1)

def set_look(yaw_atom, pitch_atom):
    yaw = float(yaw_atom.name)
    pitch = float(pitch_atom.name)
    response = _ros_set_look(yaw, pitch)
    if response == True:
        return TruthValue(1,1)
    else:
        return TruthValue(0,1)

def set_relative_look(pitch_atom, yaw_atom):
    pitch = float(pitch_atom.name)
    yaw = float(yaw_atom.name)
    response = _ros_set_relative_look(pitch, yaw)
    if response == True:
        return TruthValue(1,1)
    else:
        return TruthValue(0,1)
            
def set_relative_move(yaw_atom, dist_atom, jump_atom):
    print 'set_rel_move'
    yaw = float(yaw_atom.name)
    dist = float(dist_atom.name)
    jump = False
    response = _ros_set_relative_move(yaw, dist, jump)
    print 'set_rel_move: yaw, dist, jump, res', yaw, dist, jump, response
    if response == True:
        return TruthValue(1,1)
    else:
        return TruthValue(0,1)
