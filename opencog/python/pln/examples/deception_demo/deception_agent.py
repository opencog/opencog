"""
A MindAgent for a Simple Deception described at
http://wiki.opencog.org/w/Simple_Deception
"""

from opencog.cogserver import MindAgent
from opencog.atomspace import types
from pln.chainers import Chainer
from pln.rules import *

TARGET_STIMULUS = 20

class DeceptionAgent(MindAgent):
    def __init__(self):
        self.chainer = None
        print "Initialized DeceptionAgent."

    def create_chainer(self, atomspace, stimulate_atoms=False):
        self.chainer = Chainer(atomspace,
                               agent=self,
                               stimulateAtoms=stimulate_atoms,
                               allow_output_with_variables=False,
                               delete_temporary_variables=True)

        self.chainer.add_rule(
            ModusPonensRule(self.chainer, types.ImplicationLink))

    def run(self, atomspace):
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
        return self.chainer.history.get_history()


def check_result(atomspace):
    pass


