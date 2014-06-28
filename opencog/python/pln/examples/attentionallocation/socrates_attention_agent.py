__author__ = 'sebastian'

from opencog.cogserver import MindAgent
from opencog.atomspace import types
from pln.chainers import Chainer
from pln.rules import *


class SocratesAgent(MindAgent):
    def __init__(self):
        self.chainer = None

    def create_chainer(self, atomspace):
        self.chainer = Chainer(atomspace,
                               agent=self,
                               stimulateAtoms=True,
                               preferAttentionalFocus=True,
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
            print("PLN Chainer created.")
            return

        print("PLN continuing.")

        # there is no query here, so it doesn't give any stimulus

        if not check_result(atomspace):
            result = self.chainer.forward_step()
            return result

@staticmethod
def check_result(self, atomspace):

    result_found = False
    eval_links = atomspace.get_atoms_by_type(types.EvaluationLink)

    for eval_link in eval_links:
        out = atomspace.get_outgoing(eval_link.h)
        if out[0].is_a(types.PredicateNode) and out[0].name == "breathe"\
            and out[1].is_a(types.ListLink)\
            and "Socrates" in out[1][0]\
            and "air" in out[1][1]:
            result_found = True
            break

    print "Result found {0}".format(result_found)
    return result_found

