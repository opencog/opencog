"""
Testing PLN inference on socrates-r2l.scm input
"""

from opencog.cogserver import MindAgent
from opencog.atomspace import types
from pln.chainers import Chainer
from pln.rules import *

__author__ = 'Cosmo Harrigan'


class SocratesAgent(MindAgent):

    steps = 0

    def __init__(self):
        self.chainer = None

    def create_chainer(self, atomspace):
        self.chainer = Chainer(atomspace,
                               stimulateAtoms=False,
                               allow_output_with_variables=False,
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
            print "PLN Chainer created."
            return

        #print "PLN continuing."
        print("Step #{0}".format(self.steps))

        # there is no query for this example

        if not check_result(atomspace):
            result = self.chainer.forward_step()
            self.steps += 1
            # no query to stimulate

            return result

        # agent needs to be stopped when result is reached


def check_result(atomspace):
    """
    Searches for an instance of
    EvaluationLink
        PredicateNode "breathe"
        ListLink
            ConceptNode "Socrates"
            ConceptNode "air"
    """
    result_found = False
    eval_links = atomspace.get_atoms_by_type(types.EvaluationLink)

    for eval_link in eval_links:
        out = atomspace.get_outgoing(eval_link.h)
        if out[0].is_a(types.PredicateNode) and "breathe" in out[0].name\
            and out[1].is_a(types.ListLink)\
            and "Socrates" in out[1].out[0].name\
                and "air" in out[1].out[1].name:
            result_found = True
            break

    if result_found:
        print("Result found? {0}.".format(result_found))

    return result_found
