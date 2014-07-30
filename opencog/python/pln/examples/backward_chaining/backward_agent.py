"""
For testing backward-chaining on the criminal example
"""

from opencog.cogserver import MindAgent
from pln.chainers import Chainer
from pln.rules import *

__author__ = 'Sebastian Ruder'


class BackwardAgent(MindAgent):
    def __init__(self):
        self.chainer = None

    def create_chainer(self, atomspace):
        self.chainer = Chainer(atomspace,
                               stimulateAtoms=False,
                               allow_output_with_variables=True,
                               delete_temporary_variables=True)

        self.chainer.add_rule(
            GeneralEvaluationToMemberRule(self.chainer, 0, 2))
        self.chainer.add_rule(MemberToInheritanceRule(self.chainer))
        self.chainer.add_rule(
            DeductionRule(self.chainer, types.InheritanceLink))
        self.chainer.add_rule(
            InheritanceToMemberRule(self.chainer))
        self.chainer.add_rule(
            MemberToEvaluationRule(self.chainer))
        self.chainer.add_rule(
            AbductionRule(self.chainer, types.InheritanceLink))
        self.chainer.add_rule(
            ModusPonensRule(self.chainer, types.InheritanceLink))

    def run(self, atomspace):
        if self.chainer is None:
            self.create_chainer(atomspace)
            return

        result = self.chainer.backward_step()
        return result
