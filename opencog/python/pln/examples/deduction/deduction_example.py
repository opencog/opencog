"""
PLN Deduction Example

Demonstrates how to run the example in deduction_agent.py when
when interacting with PLN from a standalone Python environment
for development or testing purposes. The normal use case is to
run the example from the CogServer, for which you should use
deduction_agent.py instead.
"""

from __future__ import print_function
from pprint import pprint
from pln.examples.deduction import deduction_agent
from opencog.atomspace import types, AtomSpace, TruthValue

__author__ = 'Cosmo Harrigan'

# Create an AtomSpace with some sample information, equivalent to the
# information in atomspace_contents.scm
atomspace = AtomSpace()

# Basic concepts
frog = atomspace.add_node(types.ConceptNode, 'Frog', TruthValue(0.01, 100))
intelligent = atomspace.add_node(types.ConceptNode,
                                 'Intelligent',
                                 TruthValue(0.05, 100))
slimy = atomspace.add_node(types.ConceptNode, 'Slimy', TruthValue(0.01, 100))
animal = atomspace.add_node(types.ConceptNode, 'Animal', TruthValue(0.1, 100))
being = atomspace.add_node(types.ConceptNode, 'Being', TruthValue(0.1, 100))
moves = atomspace.add_node(types.PredicateNode, 'Moves', TruthValue(0.1, 100))

# Attributes of frogs
atomspace.add_link(types.InheritanceLink,
                   [frog, intelligent],
                   TruthValue(0.2, 100))
atomspace.add_link(types.InheritanceLink, [frog, slimy], TruthValue(0.5, 100))
atomspace.add_link(types.InheritanceLink, [frog, animal], TruthValue(0.9, 100))

# Attributes of animals
atomspace.add_link(types.InheritanceLink,
                   [animal, being],
                   TruthValue(0.9, 100))
atomspace.add_link(types.InheritanceLink,
                   [animal, moves],
                   TruthValue(0.9, 100))

# Peter is a frog
peter = atomspace.add_node(types.ConceptNode, 'Peter', TruthValue(0.001, 100))
atomspace.add_link(types.InheritanceLink, [peter, frog], TruthValue(0.9, 100))

#print('AtomSpace starting contents:')
#atomspace.print_list()

# Test multiple steps of forward inference on the AtomSpace
deduction_agent = deduction_agent.DeductionAgent()
for i in range(1, 500):
    result = deduction_agent.run(atomspace)

    output = None
    input = None
    rule = None
    if result is not None:
        (rule, input, output) = result

    if output is not None:
        print("\n---- [Step # {0}] ----".format(i))
        print("-- Output:\n{0}".format(output[0]))
        print("-- Rule:\n{0}".format(rule))
        print("\n-- Input:\n{0}".format(input))

print('\n--- Trails:')
trails = deduction_agent.get_trails()
i = 0
for item in trails:
    i += 1
    print("[%s]\n%s" % (i, item))

print('--- History:')
history = deduction_agent.get_history()
pprint(vars(history))

with open('pln_log.txt', 'w') as logfile:
    all_atoms = atomspace.get_atoms_by_type(t=types.Atom)
    print('; Number of atoms in atomspace after inference: %d' %
          len(all_atoms), file=logfile)
    for atom in all_atoms:
        print(atom, file=logfile)
