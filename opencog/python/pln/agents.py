__author__ = 'ramin'

from opencog.cogserver import MindAgent
from opencog.atomspace import AtomSpace, types
from logic import ForwardChainer
import random
import Rules


class SimpleForwardInferenceAgent(MindAgent):
    def run(self, atomspace=None):
        node = self.fetch(atomspace)
        fc = ForwardChainer(atomspace)
        fc.add_rule(Rules.DeductionRule(atomspace))
        fc.run(node)

    def fetch(self, atomspace=None):
        links = list(atomspace.get_atoms_by_type(types.InheritanceLink))
        r = random.randrange(0, len(links))
        return links[r].out[0]
