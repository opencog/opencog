from opencog.type_constructors import *
from opencog.bindlink import bindlink
from opencog.atomspace import types,get_refreshed_types
import opencog.spacetime
from opencog.type_constructors import *
from opencog.atomspace import Atom
types = get_refreshed_types()

"""
EvaluationLink(
GroundedPredicateNode("py: update_av")
ListLink(
VariableNode("$x"),
NumberNode("0.5")
)
)
"""

class AttentionController:
    """
    Used for simply control the attention value of atoms
    """
    def __init__(self, atomspace):
        self._atomspace = atomspace

    def control_av_in_atomspace(self):
        new_handle = bindlink(self._atomspace,
                                             BindLink(
                                                 VariableNode("$x"),
                                                 EvaluationLink(
                                                     PredicateNode("new_block"),
                                                     VariableNode("$x")
                                                 ),
                                                 EvaluationLink(
                                                     PredicateNode("new_block"),
                                                     VariableNode("$x")
                                                 )
                                             ).h)
        new_atom = Atom(new_handle, self._atomspace)
        disappeared_handle  = bindlink(self._atomspace,
                                              BindLink(
                                                  VariableNode("$x"),
                                                  EvaluationLink(
                                                      PredicateNode("disappeared"),
                                                      VariableNode("$x")
                                                  ),
                                                  EvaluationLink(
                                                      PredicateNode("disappeared"),
                                                      VariableNode("$x")
                                                  )
                                              ).h)
        disappeared_atom = Atom(disappeared_handle, self._atomspace)
        print "ac.control_av_in_atomspace: found blocks", disappeared_atom
        all_eval_links = new_atom.out + disappeared_atom.out
        print all_eval_links
        for eval_link in all_eval_links:
            atom = eval_link.out[1]
            cur_sti = atom.av['sti']
            print cur_sti
            self._atomspace.set_av(atom.h, sti = cur_sti + 5)
            print atom
            print atom.av
            self._atomspace.remove(eval_link)
        for block in self._atomspace.get_atoms_by_type(types.StructureNode):
            cur_sti = block.av['sti']
            self._atomspace.set_av(block.h, sti = cur_sti - 1)
            print block.av
