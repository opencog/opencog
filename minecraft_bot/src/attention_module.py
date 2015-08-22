# attention_module.py
#! /usr/bin/env python2.7 python2 python
"""Control/Update the attention value of all atoms in atomspace
This module should be imported in the opencog_initializer.py
to control the av in the main loop.
For now all the works are handled by AttentionContorller class

available class:
    AttentionController: used in the main loop to control av
TODO:
    Integrated with the existing attention control code:
        For now we just use a simple way(see the doc in the class)
        to control the attention value of atom. In the long run,
        we may need a more clever and complicated way to do this.
"""
from opencog.type_constructors import *
from opencog.bindlink import bindlink
from opencog.atomspace import types,get_refreshed_types
import opencog.spacetime #for import spacetime atom types
from opencog.type_constructors import *
from opencog.atomspace import Atom
types = get_refreshed_types() #must do so or types will miss spacetime types

class AttentionController:
    """Used for simply control the attention value of atoms
    control the av in control_av_in_atomspace method.
    Arg:
        atomspace(opencog.atomspace.AtomSpace): atomspace in the main loop
    """
    def __init__(self, atomspace):
        self._atomspace = atomspace

    def control_av_in_atomspace(self):
        """Called in the main loop to update av
        For now we control av in this way:
        if new block appeared or old block disappeared:
        (e.g. finding such atom structures
            EvaluationLink
                PredicateNode "new_block"
                StructureNode "objXX"
        )
            increase its av
            remove the new block/ disappeared predicate
        for all block:
            decrease their av
        """
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
        disappeared_handle = bindlink(self._atomspace,
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
        all_eval_links = new_atom.out + disappeared_atom.out
        for eval_link in all_eval_links:
            atom = eval_link.out[1]
            cur_sti = atom.av['sti']
            print cur_sti
            self._atomspace.set_av(atom.h, sti=cur_sti + 5)
            print atom
            self._atomspace.remove(eval_link)
        for block in self._atomspace.get_atoms_by_type(types.StructureNode):
            cur_sti = block.av['sti']
            self._atomspace.set_av(block.h, sti=cur_sti - 1)
            print block
