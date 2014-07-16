"""
Demo to demonstrate use of temporal reasoning pipeline
1. RelEx and RelEx2Logic process natural language sentences
2. PLN performs inferences using AI on output atoms
"""

from __future__ import print_function
from opencog.atomspace import types, AtomSpace, TruthValue
from opencog.scheme_wrapper import load_scm, scheme_eval, scheme_eval_h, __init__
from pln.examples.interactive_agent import InteractiveAgent
from opencog.cogserver import MindAgent
from pln.chainers import Chainer
from pln.rules.temporal_rules import create_temporal_rules

__author__ = 'Sebastian Ruder'


class TemporalAgent(MindAgent):
    """
    MindAgent to perform inference using Allen Interval Algebra
    """
    def __init__(self):
        self.chainer = None

    def create_chainer(self, atomspace):
        self.chainer = Chainer(atomspace,
                               stimulateAtoms=False,
                               preferAttentionalFocus=False,
                               allow_output_with_variables=True,
                               delete_temporary_variables=True)

        for rule in create_temporal_rules(self.chainer):
            self.chainer.add_rule(rule)

    def run(self, atomspace):
        if self.chainer is None:
            self.create_chainer(atomspace)
            return

        result = self.chainer.forward_step()
        return result

atomspace = AtomSpace()
__init__(atomspace)

coreTypes = "opencog/atomspace/core_types.scm"
utilities = "opencog/scm/utilities.scm"
data = "opencog/python/spatiotemporal/r2l-input.scm"

for item in [coreTypes, utilities, data]:
    load_scm(atomspace, item)

agent = InteractiveAgent(atomspace=atomspace,
                         agent=TemporalAgent(),
                         num_steps=1000,
                         print_starting_contents=True)
agent.run()
