__author__ = 'ramin'

import opencog.cogserver
from opencog.atomspace import AtomSpace, types
from logic import ForwardChainer


class fc_step(opencog.cogserver.Request):
    def run(self, args=[], atomspace=None):
        fc = ForwardChainer(atomspace)

        self.send("This is will do a forward chain soon.")