"""
Testing EvaluationToMember for correct output and examining the
SatisfyingSet behavior
"""

from opencog.cogserver import MindAgent
from opencog.atomspace import types
from pln.chainers import Chainer
from pln.rules import *

__author__ = 'Cosmo Harrigan'


class EvaluationToMemberAgent(MindAgent):
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

    def run(self, atomspace):
        if self.chainer is None:
            self.create_chainer(atomspace)
            return

        result = self.chainer.forward_step()
        return result
