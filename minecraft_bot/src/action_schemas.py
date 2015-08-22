# action_schemas.py
#! usr/bin/env python2.7 python2 python
"""
action schemas called by GroundedPredicate/SchemaNode in ActionGenerator

When importing this it will ask for the ROS service to ROS.
So to make it work we have to start the embodiment-testing/actionsnode.py
Note:
1. All function arguments must be atom. Or the GPN/GSN will fail.
   If the return value of function is not atom, the GSN will fail.
   If the return value of function is not TruthValue, the GPN will fail.
2. Because the GSN/GPN will find functions in outermost shell scope,
   we have to import schemas in starting script(e.g. opencog_initializer.py),
   not the action generator module.
   Or, the python evaluator in Opencog will fail to find the function.
3. We assume the atomspace and Space/Time server in main loop
   are the ones in the opencog.spacetime.SpaceTimeAndAtomSpace instance.
   If not, all the schemas will fail because they can't find the argument atoms.
4. For now(20150822) they are imported through the config file passed to
   opencog.utilities.initialize_opencog() in starting script. If not do so,
   the Python evaluator in Opencog cannot find function name through module,
   ex. It can find "is_attractive"
       but unable to find "action_schemas.is_attractive".
   This behavior is strange; for now to make code easier to read we want to
   put the module name(action_schema) in the GPN/GSN. So we have to import it
   in such a weird way.

Method:
is_attractive(atom): To judge an atom is attractive enough(sti is large enough).
move_toward_block(block_atom): move to a place near the input block.
set_look(pitch_atom, yaw_atom): look toward the given direction.
set_relative_look(pitch_atom, yaw_atom): look toward current + given direction.
set_relative_move(yaw_atom, dist_atom, jump_atom): move toward the given yaw.

Exception:
    rospy.ServiceException: raised when errored in asking for ROS services.

TODO:
Seperate the schemas to different modules: some schemas are for decision making
    (ex. is_attractive), and others are for executing action.
    We should put them in different modules or it will be hard to manage them.
    For now it's OK because there's only a few functions.
    And a problem is that, the initialize_opencog() can't read the config file
    correctly when we set multiple path in PYTHON_PRELOAD_FUNCTIONS. It must be
    fixed if we want to seperate schemas.
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
    """judge if atom is attractive enough
    Return: TruthValue(1,1) if it's attractive else TruthValue(0,1)
    TODO: 
    Set the sti standard value as argument: For now we set "sti > 1" as condition.
    """
    print 'is_attractive'
    sti = atom.av['sti']
    if sti > 1:
        print 'attractive!'
        return TruthValue(1,1)
    else:
        print 'boring'
        return TruthValue(0,1)

def move_toward_block(block_atom):
    """Make bot move near the block
    If there's no enough surrounding block info,
    the bot may find no place to stand on and stop moving.
    Also if there's no empty place near ( distance <= 2) the block,
    the bot will not move.
    Arg:
        block_atom(opencog.atomspace.Atom): The atom representing block
    Return: TruthValue(1,1) if move success else TruthValue(0,1)
    TODO:
        Make it faster: the calculation of near place is slow.
        Add distance argument: make caller control the judgement of "near"
    """
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

def set_look(pitch_atom, yaw_atom):
    """set look toward the given direction.
    The look direction will be changed to the diretion we pass.
    Args:
        pitch_atom(opencog.atomspace.Atom): A NumberNode representing pitch in degree
        yaw_atom(opencog.atomspace.Atom): A NumberNode representing yaw in degree
    Return:  TruthValue(1,1) if set look success else TruthValue(0,1)
    """
    pitch = float(pitch_atom.name)
    yaw = float(yaw_atom.name)
    response = _ros_set_look(yaw, pitch)
    if response == True:
        return TruthValue(1,1)
    else:
        return TruthValue(0,1)

def set_relative_look(pitch_atom, yaw_atom):
    """set relative look toward the given direction.
    The look will be changed by adding pitch and yaw on current direction.
    Args:
        pitch_atom(opencog.atomspace.Atom):
            A NumberNode representing relative pitch in degree
        yaw_atom(opencog.atomspace.Atom):
            A NumberNode representing relative yaw in degree
    Return:  TruthValue(1,1) if set look success else TruthValue(0,1)
    """

    pitch = float(pitch_atom.name)
    yaw = float(yaw_atom.name)
    response = _ros_set_relative_look(pitch, yaw)
    if response == True:
        return TruthValue(1,1)
    else:
        return TruthValue(0,1)
            
def set_relative_move(yaw_atom, dist_atom, jump_atom):
    """set relative move toward the given direction.
    The bot will move toward the yaw direction in input distance.
    For now the jump_atom is not used. We assume bot will not jump
    during the move.
    Args:
        yaw_atom(opencog.atomspace.Atom):
            A NumberNode representing yaw in degree
        dist_atom(opencog.atomspace.Atom):
            A NumberNode representing distance in Minecraft unit
        jump_atom(opencog.atomspace.Atom):
            not used now, should be used for determining jump or not.
    Return: TruthValue(1,1) if move success else TruthValue(0,1)
    TODO:
        use jump_atom to determine jump or not: We can determine jump
        or not by checking the TV of jump_atom.
    """

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
