"""
For testing smokes_agent.py without the cogserver
"""

from __future__ import print_function
from pln.examples.tuffy.smokes.smokes_agent import InferenceAgent
from opencog.atomspace import AtomSpace, TruthValue, types
from data import *
from pln.examples.interactive_agent import *

__author__ = 'Cosmo Harrigan'

agent = InteractiveAgent(atomspace=atomspace,
                         agent=InferenceAgent(),
                         num_steps=500,
                         print_starting_contents=True)
agent.run()

print("\n---- Results:\n")

# EvaluationLinks where the first argument is PredicateNode "cancer"
# and the target of the predicate is a ConceptNode (representing a person)
eval_links = atomspace.get_atoms_by_type(types.EvaluationLink)
for eval_link in eval_links:
    out = [atom for atom in atomspace.get_outgoing(eval_link.h)
           if atom.is_a(types.PredicateNode) and atom.name == "cancer"]
    if out:
        list_link = atomspace.get_outgoing(eval_link.h)[1]
        argument = atomspace.get_outgoing(list_link.h)[0]
        if argument.is_a(types.ConceptNode):
            print(eval_link)
