__author__ = 'sebastian'

"""
For running the deception_agent
"""

from pln.examples.deception_demo import deception_agent
from opencog.atomspace import AtomSpace
from opencog.scheme_wrapper import load_scm, __init__
from pln.examples.interactive_agent import InteractiveAgent

atomspace = AtomSpace()
__init__(atomspace)

coreTypes = "opencog/atomspace/core_types.scm"
utilities = "opencog/scm/utilities.scm"
path = "opencog/python/pln/examples/deception_demo/"
data = [path + "deception.scm", path + "deception_demo/",
        path + "spatial_information.scm"]

number_of_steps = 100

for item in [coreTypes, utilities] + data:
    load_scm(atomspace, item)

agent = InteractiveAgent(atomspace=atomspace,
                         agent=deception_agent.DeceptionAgent(),
                         num_steps=number_of_steps,
                         print_starting_contents=True)

agent.run()
