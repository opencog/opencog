"""
For testing smokes_agent.py without the cogserver
"""

from __future__ import print_function
from opencog.atomspace import AtomSpace, TruthValue, types
from opencog.scheme_wrapper import load_scm, scheme_eval, scheme_eval_h, __init__
from pln.examples.tuffy.smokes.smokes_agent import InferenceAgent, check_result

__author__ = 'Cosmo Harrigan'

# Set to True to include extra, un-needed data, to evaluate inference control
# efficiency
EXTRA_DATA = True

atomspace = AtomSpace()
__init__(atomspace)

data = ["opencog/atomspace/core_types.scm",
        "opencog/scm/utilities.scm",
        "opencog/python/pln/examples/tuffy/smokes/smokes.scm"]

if EXTRA_DATA:
    data.append("opencog/python/pln/examples/tuffy/smokes/extra-data.scm")

for item in data:
    load_scm(atomspace, item)

atoms = atomspace.get_atoms_by_type(types.Atom)
for atom in atoms:
    print(atom)
MAX_STEPS = 500

chainer = InferenceAgent()
chainer.create_chainer(atomspace=atomspace, stimulate_atoms=False)

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

    if check_result(atomspace):
        answer = True
        break

if answer:
    print("\n---- Answer found after {0} inference steps that produced a new "
          "output, out of {1} total inference steps attempted.".
          format(outputs_produced, i+1))
else:
    print("No result")
