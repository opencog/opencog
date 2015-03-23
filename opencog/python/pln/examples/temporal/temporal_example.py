"""
Demo to demonstrate use of temporal reasoning pipeline
1. RelEx and RelEx2Logic process natural language sentences
2. PLN performs inferences using AI on output atoms
"""

from __future__ import print_function
from opencog.atomspace import types, AtomSpace, TruthValue
from opencog.scheme_wrapper import load_scm, scheme_eval, scheme_eval_h, __init__
from pln.chainers import Chainer
from pln.rules.temporal_rules import create_temporal_rules

__author__ = 'Sebastian Ruder'

num_steps = 100
print_starting_contents = True
coreTypes = "opencog/atomspace/core_types.scm"
utilities = "opencog/scm/utilities.scm"
timeLinks = "opencog/spacetime/spacetime_types.scm"
data = "tests/python/test_pln/scm_disabled/temporal/temporalToyExample.scm"
# data = "opencog/python/pln/examples/temporal/temporal-r2l-input.scm"

# initialize atomspace
atomspace = AtomSpace()
__init__(atomspace)
for item in [coreTypes, utilities, timeLinks, data]:
    load_scm(atomspace, item)

# initialize chainer
chainer = Chainer(atomspace,
                  stimulateAtoms=False,
                  allow_output_with_variables=True,
                  delete_temporary_variables=True)
for rule in create_temporal_rules(chainer):
    chainer.add_rule(rule)

if print_starting_contents:
    print('AtomSpace starting contents:')
    atomspace.print_list()

outputs_produced = 0

for i in range(0, num_steps):
    result = chainer.forward_step()

    output = None
    input = None
    rule = None
    if result is not None:
        atomspace_string = ""
        (rule, input, output) = result
        outputs_produced += 1

        print("\n----- [Output # {0}] -----".format(outputs_produced))
        for j in output:
            print("-- Output:\n{0}".format(j))
        print("-- using production rule: {0}".format(rule.name))
        print("\n-- based on this input:\n{0}".format(input))
