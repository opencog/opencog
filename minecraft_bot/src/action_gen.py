from opencog.atomspace import types, TruthValue
from opencog.type_constructors import *
from opencog.bindlink import bindlink, evaluate_atom
from opencog.atomspace import Atom
class ActionGenerator:

    """
    for testing, we set a target block(gold, which block id is 14)
    when generating action it will find the target block at first,
    and check if its sti is large enough. If so, move toward it(assume it's superflat world),
    If no result, change look or random move.
    """
    def __init__(self, atomspace, space_server, time_server):
        self.atomspace = atomspace
        self.space_server = space_server
        self.time_server = time_server

    def generate_action(self):
        result = bindlink(self.atomspace,
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
        print "action_gen: result", Atom(result, self.atomspace)
        
        if self.atomspace.get_outgoing(result) == []:
            #TODO: fix the multiple atomspace bug 
            #between execute_atom and bindlink
            print "action_gen: no result, random walk."
            
            evaluate_atom(self.atomspace,
                             EvaluationLink(
                                 GroundedPredicateNode("py: action_schemas.set_relative_move"),
                                 ListLink(
                                     NumberNode("90"),
                                     NumberNode("2"),
                                     ConceptNode("jump")
                                 )
                             )
                         )
            print "action_gen: random walk end"
        
                    
                
