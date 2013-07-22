__author__ = 'ramin'

from opencog.atomspace import AtomSpace, types, TruthValue
from agents import SimpleForwardInferenceAgent


atomspace = AtomSpace()

a = atomspace.add_node(types.ConceptNode, "ramin", TruthValue(1, 1))
b = atomspace.add_node(types.ConceptNode, "keyvan", TruthValue(1, 1))
c = atomspace.add_node(types.ConceptNode, "ashkan", TruthValue(1, 1))

ab = atomspace.add_link(types.InheritanceLink, [a, b], TruthValue(1, 1))
ac = atomspace.add_link(types.InheritanceLink, [b, c], TruthValue(1, 1))

agent = SimpleForwardInferenceAgent()
agent.run(atomspace)

atomspace.print_list()

