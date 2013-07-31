__author__ = 'ramin'

from opencog.cogserver import MindAgent
from opencog.atomspace import AtomSpace, types
from logic import ForwardChainer
import random
import rules


class SimpleForwardInferenceAgent(MindAgent):
    def run(self, atomspace=None):
        node = self.fetch(atomspace)
        fc = ForwardChainer(atomspace)
        fc.add_rule(rules.DeductionRule(atomspace))
        fc.run(node)

    def fetch(self, atomspace=None):
        links = list(atomspace.get_atoms_by_type(types.InheritanceLink))
        return self._selectOne(links)

    def _selectOne(self, links):
        max = sum([link.getav().sti for link in links])
        pick = random.uniform(0, max)
        current = 0
        for link in links:
            current += link.getav().sti
            if current >= pick:
                return link
