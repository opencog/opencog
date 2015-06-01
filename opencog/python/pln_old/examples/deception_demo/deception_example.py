__author__ = 'sebastian, amen'

"""
For running the deception_agent
"""
from pln.rules.boolean_rules import simplify_boolean
from pln.examples.deception_demo import deception_agent
from opencog.atomspace import AtomSpace
from opencog.scheme_wrapper import load_scm, __init__
from pln.examples.interactive_agent import InteractiveAgent

atomspace = AtomSpace()
__init__(atomspace)

coreTypes = "opencog/scm/core_types.scm"
embodimentTypes = "opencog/spacetime/spacetime_types.scm"
utilities = "opencog/scm/utilities.scm"
path = "opencog/python/pln_old/examples/deception_demo/"
data = [path + "deception.scm", path + "spatial_information.scm",
        path + "spatial_rules.scm", path + "agent_behavior.scm"]
#data = [path + "deception.scm"]
number_of_steps = 50

if __name__ == '__main__':
    mode = raw_input("Enter \"1\" for DECEPTION-INFERENCE, any other number for"
                     + " DECEPTION-BEHAVIOR: ")
    print mode
    if int(mode) != 1:
        number_of_steps = 500
        data += [path + "extra_data.scm"]

    for item in [coreTypes, utilities, embodimentTypes] + data:
        load_scm(atomspace, item)
    agent = InteractiveAgent(atomspace=atomspace,
                             agent=deception_agent.DeceptionAgent(int(mode)),
                             num_steps=number_of_steps,
                             print_starting_contents=False)

    agent.run()
