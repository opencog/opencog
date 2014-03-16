"""
For testing smokes_agent.py without the cogserver
"""

from __future__ import print_function
from pprint import pprint
from pln.examples.tuffy.smokes.smokes_agent import InferenceAgent
from opencog.atomspace import types, AtomSpace, TruthValue

__author__ = 'Cosmo Harrigan'

atomspace = AtomSpace()

# Anna smokes.
Anna = atomspace.add_node(types.ConceptNode, "Anna")
smokes = atomspace.add_node(types.PredicateNode, "smokes")
atomspace.add_link(types.EvaluationLink,
                   [smokes, atomspace.add_link(types.ListLink, [Anna])],
                   TruthValue(1, TruthValue().confidence_to_count(1)))

# If X smokes, then X has cancer.
cancer = atomspace.add_node(types.PredicateNode, "cancer")

## Wow -- the syntax of these bindings is extremely verbose.
X_smokes = atomspace.add_link(
    types.EvaluationLink,
    [smokes,
     atomspace.add_link(
         types.ListLink,
         [atomspace.add_node(
             types.VariableNode, "$X")])],
    TruthValue(0, TruthValue().confidence_to_count(0)))

X_cancer = atomspace.add_link(
    types.EvaluationLink,
    [cancer,
     atomspace.add_link(
         types.ListLink,
         [atomspace.add_node(
             types.VariableNode, "$X")])],
    TruthValue(0, TruthValue().confidence_to_count(0)))

atomspace.add_link(
    types.ImplicationLink,
    [X_smokes,
     X_cancer],
    TruthValue(1, TruthValue().confidence_to_count(1)))

print_starting_contents = True
if print_starting_contents:
    print('AtomSpace starting contents:')
    atomspace.print_list()

# Test multiple steps of forward inference on the AtomSpace
agent = InferenceAgent()
outputs_produced = 0

for i in range(1, 500):
    result = agent.run(atomspace)

    output = None
    input = None
    rule = None
    if result is not None:
        (rule, input, output) = result
        outputs_produced += 1

    if result is not None:
        print("\n--------- [Output # {0}] ---------".format(outputs_produced))
        print("-- Output:\n{0}".format(output[0]))
        print("-- using production rule: {0}".format(rule.name))
        print("\n-- based on this input:\n{0}".format(input))

with open('pln_log.txt', 'w') as logfile:
    all_atoms = atomspace.get_atoms_by_type(t=types.Atom)
    print('; Number of atoms in atomspace after inference: %d' %
          len(all_atoms), file=logfile)
    for atom in all_atoms:
        print(atom, file=logfile)
