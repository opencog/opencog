__author__ = 'ramin'

from opencog.atomspace import AtomSpace
from agents import SimpleForwardInferenceAgent, AtomspacePopulatorAgent




atomspace = AtomSpace()

fia_agent = SimpleForwardInferenceAgent()
apa_agent = AtomspacePopulatorAgent()

for i in range(1, 100):
    apa_atom = apa_agent.run(atomspace)
    if apa_atom is not None:
        print 'apa:', apa_atom
    fia_atom = fia_agent.run(atomspace)
    if fia_atom is not None:
        print 'fia:', fia_atom

print "----------------------------------------"
atomspace.print_list()

