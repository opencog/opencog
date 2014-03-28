"""
PLN representation of the "RC/class" sample from Tuffy Markov Logic Networks

More details on this sample are available here:
https://github.com/cosmoharrigan/tuffy/tree/master/samples/class
http://arxiv.org/pdf/1104.3216.pdf

Instructions are located here:
https://github.com/opencog/test-datasets/blob/master/pln/tuffy/class/tests/README.md
"""

from opencog.cogserver import MindAgent
from opencog.atomspace import types
from pln.chainers import Chainer
from pln.rules import *

__author__ = 'Cosmo Harrigan'


class InferenceAgent(MindAgent):
    def __init__(self):
        self.chainer = None
        self.antecedents = []

    def create_chainer(self, atomspace):
        self.chainer = Chainer(atomspace,
                               stimulateAtoms=True,
                               preferAttentionalFocus=True,
                               delete_temporary_variables=True,
                               allow_output_with_variables=True)

        self.chainer.add_rule(
            ModusPonensRule(self.chainer, types.ImplicationLink))

    def run(self, atomspace):
        if self.chainer is None:
            self.create_chainer(atomspace)

            implications = atomspace.get_atoms_by_type(types.ImplicationLink)
            for implication in implications:
                outgoing = atomspace.get_outgoing(implication.h)
                and_link = outgoing[0]
                self.antecedents.append(and_link)

            return

        # Run a forward step of the chainer with the ModusPonensRule so as to
        # perform inference using the "domain rules" defined as
        # ImplicationLinks
        self.chainer.forward_step()

        # Run a backward step of the chainer with the AndCreationRule for each
        # of the antecedents of the defined "domain rules" so as to
        # successfully ground the variables.
        for antecedent in self.antecedents:
            num_predicates = len(atomspace.get_outgoing(antecedent.h))
            self.chainer.backward_step(
                AndCreationRule(self.chainer, num_predicates), [antecedent])

        return
