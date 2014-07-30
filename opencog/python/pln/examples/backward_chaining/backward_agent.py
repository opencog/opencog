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
            ModusPonensRule(self.chainer, types.ImplicationLink))
        #self.chainer.add_rule(
         #   AndCreationRule(self.chainer, 2))

    def run(self, atomspace):
        if self.chainer is None:
            self.create_chainer(atomspace)
            print "PLN Chainer created."
            return

        #if not check_result(atomspace):
        result = self.chainer.backward_step()
        return result


def check_result(atomspace):
    """
    Searches for
    (InheritanceLink
        (VariableNode "$isCriminal")
        (ConceptNode "criminal")))
    """
    inh_links = atomspace.get_atoms_by_type(types.InheritanceLink)
    result_found = False

    for inh_link in inh_links:
        args = atomspace.get_outgoing(inh_link.h)
        if args[0].is_a(types.ConceptNode) and args[1].name == "criminal":
            criminal = args[0].name
            result_found = True
            print inh_link
    if result_found:
        print("Suspect found guilty! {0} is a criminal!".format(criminal))
    return result_found
