# action_gen.py
#! /usr/bin/env python2.7 python2 python
""" module to generate action in Opencog

This is the part to decide and execute action in Minecraft embodiment.
For now all the work is handled by ActionGenerator class.
It will determine the behavior by a behavior tree and
execute this behavior by calling GPN/GSN.

TODO:
In the future we should seperate the decision making
and action execution to make the ActionGenerator class cleaner.
Also we should have a more systematic way to handle decision,
not just determining behavior by if-else statement.

available class:
    ActionGenerator: used in the main loop of Opencog bot to decide action
"""
from opencog.atomspace import types, TruthValue
from opencog.type_constructors import *
from opencog.bindlink import bindlink, evaluate_atom
from opencog.atomspace import Atom
class ActionGenerator:
    """ determining and executing action of Opencog bot in each loop

    The main loop will call the generate_action() to do all the work.

    Args:
        atomspace(opencog.atomspace.AtomSpace): the AtomSpace in main loop
        space_server(opencog.spacetime.SpaceServer): the SpaceServer in main loop
        time_server(opencog.spacetime.TimeServer): the TimeServer in main loop
    Method:
        generate_action(): generate and execute the action by behavior tree
    """
    def __init__(self, atomspace, space_server, time_server):
        self._atomspace = atomspace
        self._space_server = space_server
        self._time_server = time_server

    def generate_action(self):
        """ generate and execute the action by behavior tree

        Now (20150822) we generate action by such a behavior tree:

        If target block found in atomspace:
            If target block is attractive(sti is large enough):
                move to the near place of the target block
        else:
            moving 2 units toward yaw = 90 degree

        For testing, we set a target block(gold, which block id is 14)
        And we assume the world is superflat world so we only need to move on
        x-y direction.
        Note that because the behavior of GSN/GPN,
        we have to import all schemas(action_schemas.py)
        in the main script(opencog_initializer.py).
        Or it will fail to find the function name.

        TODO:
            Build random action function: It's for the 'else' part
                in above behavior tree. This will make the bot behaves more
                naturally.
            Add pathfinding technique: so the bot can calculate the procedure to
                get to the destination.
                it will be able to move to any place he wants.
                (ex. place block to jump to a higher place)
                In Opencog there has been an OCPlanner implementation.
                It should be possible to migrate the old code.
            More higher level ways to making decision: For now it's just testing
                so we only use a simple behavior tree to demo. But in general,
                we should regard these behavior tree as lower level schemas.
                And we should use a higher level cognition ways(e.g. OpenPsi)
                to decide what behavior tree we want to execute.
        """
        result = bindlink(self._atomspace,
                          BindLink(
                              VariableNode("$block"),
                              AndLink(
                                  EvaluationLink(
                                      PredicateNode("material"),
                                      ListLink(
                                          VariableNode("$block"),
                                          ConceptNode("14")
                                      )
                                  ),
                                  EvaluationLink(
                                      GroundedPredicateNode("py: action_schemas.is_attractive"),
                                      ListLink(
                                          VariableNode("$block")
                                      )
                                  ),
                                  EvaluationLink(
                                      GroundedPredicateNode("py: action_schemas.move_toward_block"),
                                      ListLink(
                                          VariableNode("$block")

                                      )
                                  )
                              ),
                              VariableNode("$block")
                          ).h
                      )
        print "action_gen: result", Atom(result, self._atomspace)
        
        if self._atomspace.get_outgoing(result) == []:
            print "action_gen: no result, random walk."

            evaluate_atom(self._atomspace,
                             EvaluationLink(
                                 GroundedPredicateNode("py: action_schemas.set_relative_move"),
                                 ListLink(
                                     NumberNode("90"),
                                     NumberNode("2"),
                                     ConceptNode("jump")
                                 )
                             )
                         )

