__author__ = 'sebastian'

# Socrates example is a categorical syllogism --> move in this directory?

"""
For testing syllogisms
"""

from __future__ import print_function
from opencog.atomspace import types, AtomSpace, TruthValue
from opencog.scheme_wrapper import load_scm, scheme_eval, scheme_eval_h, __init__
from pln.examples.interactive_agent import InteractiveAgent
from opencog.cogserver import MindAgent
from opencog.atomspace import types
from pln.chainers import Chainer
from pln.rules import *

__author__ = 'Sebastian Ruder'


class SyllogismAgent(MindAgent):
    def __init__(self):
        self.chainer = None

    def create_chainer(self, atomspace):
        self.chainer = Chainer(atomspace,
                               stimulateAtoms=False,
                               allow_output_with_variables=True,
                               delete_temporary_variables=True)

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
data = "opencog/python/pln/examples/relex2logic/socrates-r2l.scm"

for item in [coreTypes, utilities, data]:
    load_scm(atomspace, item)

agent = InteractiveAgent(atomspace=atomspace,
                         agent=SyllogismAgent(),
                         num_steps=100,
                         print_starting_contents=True)
agent.run()

# iterate over input file, only load inputs
# load outputs in separate atomspace and compare them against each other
# only load rules that are specified