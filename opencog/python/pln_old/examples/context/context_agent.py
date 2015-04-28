"""
A MindAgent to test the application of the context rules
"""

from opencog.cogserver import MindAgent
from pln.chainers import Chainer
from pln.rules import *

__author__ = 'Sebastian Ruder'


class ContextAgent(MindAgent):
    def __init__(self):
        self.chainer = None

    def create_chainer(self, atomspace):
        self.chainer = Chainer(atomspace,
                               stimulateAtoms=False,
                               preferAttentionalFocus=False,
                               allow_output_with_variables=True,
                               delete_temporary_variables=True)

        self.chainer.add_rule(InheritanceToContextRule(self.chainer))
        self.chainer.add_rule(EvaluationToContextRule(self.chainer))
        self.chainer.add_rule(SubsetToContextRule(self.chainer))
        self.chainer.add_rule(ContextToInheritanceRule(self.chainer))
        self.chainer.add_rule(ContextToEvaluationRule(self.chainer))
        self.chainer.add_rule(ContextToSubsetRule(self.chainer))
        self.chainer.add_rule(ContextFreeToSensitiveRule(self.chainer))

    def run(self, atomspace):
        if self.chainer is None:
            self.create_chainer(atomspace)
            return

        result = self.chainer.forward_step()
        return result
