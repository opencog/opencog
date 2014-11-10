"""
A MindAgent for a Simple Deception described at
http://wiki.opencog.org/w/Simple_Deception
"""

from opencog.cogserver import MindAgent
from opencog.atomspace import types
from pln.chainers import Chainer
from pln.rules import *

class DeceptionAgent(MindAgent):
    def __init__(self):
        self.chainer = None
        self.query = None
        print "Initialized DeceptionAgent."

    def create_chainer(self, atomspace, stimulate_atoms=False):
        self.chainer = Chainer(atomspace,
                               agent=self,
                               stimulateAtoms=False,
                               preferAttentionalFocus=False,
                               allow_output_with_variables=True,
                               delete_temporary_variables=True)

        # Transformation rules
        # This rules are invoked randomly and are run in the background because,
        # they imply the creation of AndLinks between different sets of Links so
        # as to represent the existance of a state in the world or of state of knowledge
        # being simulated. In the ure this wouldn't be required as AndLink is used to denote
        # the existance of the required set of Links without the need of creating
        # the AndLink. If one wants to match AndLinked set of Links then the QuoteLink
        # can be used.(This is based on the assumption that ure doesn't make a fundamental
        # change on how the pattern matcher works presently)
        self.chainer.add_rule(AndCreationRule(self.chainer, 2, types.EvaluationLink))
        self.chainer.add_rule(AndCreationRule(self.chainer, 3, types.EvaluationLink))
        self.chainer.add_rule(AndCreationRule(self.chainer, 2))

    def run(self, atomspace):
        if self.chainer is None:
            self.create_chainer(atomspace)
            return

        # This insures that one of the added rules are invoked in the background
        # on each step.
        self.chainer.forward_step()

        # Any rule that is used below is assured to be executed on each step
        self.chainer.forward_step(AndCreationRule(self.chainer, 2)) #

        # Depending on choice of demo, either the result on deception reasoning
        # or the behavior that result because of the reasoning can be enabled
        # * Enable output of reasoning on rules
        result = self.chainer.forward_step(ImplicationAndRule(self.chainer))
        # * Enable ouput of behavior
#        result = self.chainer.forward_step(ModusPonensRule(self.chainer, types.ImplicationLink))

        return result

    def get_trails(self):
        return self.chainer.trails

    def get_history(self):
        return self.chainer.history.get_history()


def check_result(atomspace):
    # TODO : check the atomspace for the expected output
    pass


