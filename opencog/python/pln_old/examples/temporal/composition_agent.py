"""
A MindAgent to test the application of the temporal rules,
specifically the composition of Allen Interval relations
"""

from opencog.cogserver import MindAgent
from pln.chainers import Chainer
from pln.rules.temporal_rules import create_composition_rules

__author__ = 'Sebastian Ruder'

composition_table = "composition_table.txt"


class CompositionAgent(MindAgent):
    def __init__(self):
        self.chainer = None

    def create_chainer(self, atomspace):
        self.chainer = Chainer(atomspace,
                               stimulateAtoms=False,
                               preferAttentionalFocus=False,
                               allow_output_with_variables=True,
                               delete_temporary_variables=True)

        for rule in create_composition_rules(self.chainer, composition_table):
            self.chainer.add_rule(rule)
            print(rule.name)

    def run(self, atomspace):
        if self.chainer is None:
            self.create_chainer(atomspace)
            return

        result = self.chainer.forward_step()
        return result
