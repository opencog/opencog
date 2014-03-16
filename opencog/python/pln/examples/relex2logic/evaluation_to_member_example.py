"""
For running evaluation_to_member_agent.py without the cogserver
"""

from __future__ import print_function
from pln.examples.relex2logic import evaluation_to_member_agent
from opencog.atomspace import types, AtomSpace, TruthValue

__author__ = 'Cosmo Harrigan'

atomspace = AtomSpace()

Socrates = atomspace.add_node(types.ConceptNode, "Socrates")
Man = atomspace.add_node(types.ConceptNode, "man")
Air = atomspace.add_node(types.ConceptNode, "air")
be = atomspace.add_node(types.PredicateNode, "be")
breathe = atomspace.add_node(types.PredicateNode, "breathe")

# Todo: The Relex2Logic rules require modification to properly translate
# this relationship; see:
# https://github.com/opencog/opencog/issues/530
# Currently, it is represented the same way that Relex2Logic outputs it,
# which is not going to work properly.
atomspace.add_link(
    types.EvaluationLink,
    [be,
     atomspace.add_link(
         types.ListLink,
         [Socrates, Man])],
    TruthValue(1, TruthValue().confidence_to_count(1)))

atomspace.add_link(
    types.EvaluationLink,
    [breathe,
     atomspace.add_link(
         types.ListLink, [Man, Air])],
    TruthValue(1, TruthValue().confidence_to_count(1)))

# Example predicate with only 1 argument
#
# Anna = atomspace.add_node(types.ConceptNode, "Anna")
# smoke = atomspace.add_node(types.PredicateNode, "smoke")
#
# atomspace.add_link(
#     types.EvaluationLink,
#     [smoke,
#      atomspace.add_link(
#          types.ListLink,
#          [Anna])],
#     TruthValue(1, TruthValue().confidence_to_count(1)))

print_starting_contents = True
if print_starting_contents:
    print('AtomSpace starting contents:')
    atomspace.print_list()

# Todo: Encapsulate this agent invocation routine for reuse
agent = evaluation_to_member_agent.EvaluationToMemberAgent()
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
