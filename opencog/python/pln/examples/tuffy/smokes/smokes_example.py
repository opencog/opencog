"""
For testing smokes_agent.py without the cogserver
"""

from __future__ import print_function
from opencog.atomspace import AtomSpace, TruthValue, types
from opencog.scheme_wrapper import load_scm, scheme_eval, scheme_eval_h, __init__
from pln.examples.tuffy.smokes.smokes_agent import InferenceAgent

__author__ = 'Cosmo Harrigan'

atomspace = AtomSpace()
__init__(atomspace)

coreTypes = "opencog/atomspace/core_types.scm"
utilities = "opencog/scm/utilities.scm"
data = "opencog/python/pln/examples/tuffy/smokes/smokes.scm"

for item in [coreTypes, utilities, data]:
    load_scm(atomspace, item)

atoms = atomspace.get_atoms_by_type(types.Atom)
for atom in atoms:
    print(atom)
MAX_STEPS = 500

chainer = InferenceAgent()
chainer.create_chainer(atomspace=atomspace)


def check_result():
    # Searches for EvaluationLinks where the first argument is: PredicateNode
    # "cancer" and the target of the predicate is a ConceptNode (representing a
    # person)
    eval_links = atomspace.get_atoms_by_type(types.EvaluationLink)

    num_results = 0
    for eval_link in eval_links:
        out = [atom for atom in atomspace.get_outgoing(eval_link.h)
               if atom.is_a(types.PredicateNode) and atom.name == "cancer"]
        if out:
            list_link = atomspace.get_outgoing(eval_link.h)[1]
            argument = atomspace.get_outgoing(list_link.h)[0]
            if argument.is_a(types.ConceptNode):
                num_results += 1

    return num_results == 4

answer = False
outputs_produced = 0

for i in range(0, MAX_STEPS):
    result = chainer.run(atomspace)

    output = None
    input = None
    rule = None
    if result is not None:
        (rule, input, output) = result
        outputs_produced += 1

    if result is not None:
        print("\n----- [Output # {0}] -----".format(outputs_produced))
        print("-- Output:\n{0}".format(output[0]))
        print("-- using production rule: {0}".format(rule.name))
        print("\n-- based on this input:\n{0}".format(input))

    if check_result():
        answer = True
        break

if answer:
    print("\n---- Answer found after {0} inference steps that produced a new "
          "output, out of {1} total inference steps attempted.".
          format(outputs_produced, i+1))
else:
    print("No result")
