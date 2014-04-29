"""
PLN Simple Deduction Agent Example

Demonstrates the simplest possible forward inference agent that implements
a chainer with one inference rule and one link type

For instructions, refer to the README for PLN.
"""

from opencog.cogserver import MindAgent
from opencog.atomspace import types
from pln.chainers import Chainer
from pln.rules import *

__author__ = 'Cosmo Harrigan'

__VERBOSE__ = False


class DeductionAgent(MindAgent):
    def __init__(self):
        self.chainer = None

    def create_chainer(self, atomspace):
        self.chainer = Chainer(atomspace, stimulateAtoms=False)

        link_types = [types.InheritanceLink]

        for link_type in link_types:
            self.chainer.add_rule(DeductionRule(self.chainer, link_type))

    def run(self, atomspace):
        # Run is invoked on every cogserver cognitive cycle
        # If it is the first time it has been invoked, then the chainer
        # needs to be created
        if self.chainer is None:
            self.create_chainer(atomspace)
            return

        result = self.chainer.forward_step()

        if __VERBOSE__:
            print result

        return result

    def get_trails(self):
        return self.chainer.trails

    def get_history(self):
        return self.chainer.history_index
