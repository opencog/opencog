"""
For running the TemporalAgent without the cogserver
"""

from __future__ import print_function
from pln.examples.temporal import temporal_agent
from opencog.atomspace import types, AtomSpace, TruthValue
from opencog.scheme_wrapper import load_scm, scheme_eval, scheme_eval_h, __init__
from pln.examples.interactive_agent import InteractiveAgent

__author__ = 'Sebastian Ruder'

atomspace = AtomSpace()
__init__(atomspace)

coreTypes = "opencog/atomspace/core_types.scm"
utilities = "opencog/scm/utilities.scm"
data = ""

for item in [coreTypes, utilities, data]:
    load_scm(atomspace, item)

agent = InteractiveAgent(atomspace=atomspace,
                         agent=temporal_agent.TemporalAgent(),
                         num_steps=1000,
                         print_starting_contents=True)

agent.run()