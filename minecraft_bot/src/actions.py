import roslib; roslib.load_manifest('minecraft_bot')
import rospy
from minecraft_bot.srv import look_srv, rel_move_srv, abs_move_srv
from opencog.spacetime import SpaceTimeAndAtomSpace
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
    if atomspace.get_av(atom.h) > 1:
        print 'attractive!'
        return TruthValue(1,1)
    else:
        print 'boring'
        return TruthValue(0,1)

def move_toward(block_atom):
    print 'move toward atom', block_atom
    jump = True
    map_handle = (atomspace.get_atoms_by_name(
        types.SpaceMapNode, "MCmap")[0]).h
    cur_map = space_server.get_map(map_handle)
    block_pos = cur_map.get_block_location(block_atom.h)
    response = _ros_set_move(block_pos[0], block_pos[1], block_pos[2], jump)
    if response == True:
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
    jump = True if jump_atom.tv.mean > 0.5 else False
    response = _ros_set_relative_move(yaw, dist, jump)
    print 'set_rel_move: yaw, dist, jump, res', yaw, dist, jump, response
    if response == True:
        return TruthValue(1,1)
    else:
        return TruthValue(0,1)
