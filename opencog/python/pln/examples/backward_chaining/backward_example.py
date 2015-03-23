"""
For running evaluation_to_member_agent.py without the cogserver
"""

from __future__ import print_function
from pln.examples.backward_chaining.backward_agent import BackwardAgent, check_result
from opencog.atomspace import types, AtomSpace, TruthValue
from opencog.scheme_wrapper import load_scm, scheme_eval, scheme_eval_h, __init__
from pln.examples.interactive_agent import InteractiveAgent

__author__ = 'Sebastian Ruder'

atomspace = AtomSpace()
__init__(atomspace)

coreTypes = "opencog/atomspace/core_types.scm"
utilities = "opencog/scm/utilities.scm"
data = "opencog/python/pln/examples/backward_chaining/criminal.scm"

for item in [coreTypes, utilities, data]:
    load_scm(atomspace, item)

atoms = atomspace.get_atoms_by_type(types.Atom)
for atom in atoms:
    print(atom)
MAX_STEPS = 500

chainer = BackwardAgent()
chainer.create_chainer(atomspace=atomspace)

result_found = False
outputs_produced = 0

for i in range(MAX_STEPS):
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
        print("-- after {0} inferences attempted so far".format(i))
        print("\n-- based on this input:\n{0}".format(input))

    if check_result(atomspace):
        result_found = True
        break

if result_found:
    print("---- Answer found after {0} inference steps that produced a new "
          "output, out of {1} total inference steps attempted.".
          format(outputs_produced, i+1))
else:
    print("No result")
