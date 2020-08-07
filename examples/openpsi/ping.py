import time

from opencog.openpsi import *
from opencog.type_constructors import *

# Initialize OpenPsi object
atomspace = AtomSpace()
set_default_atomspace(atomspace)
op = OpenPsi(atomspace)

# Ball states
neutral = ConceptNode("neutral")
pinged = ConceptNode("pinged")

# Define the initial state of the ball.
ball = ConceptNode("ball")
StateLink(ball, neutral)

state_var = VariableNode("state")

# ----------------------------------------------------------------------
# DEFINING PINGING
# ----------------------------------------------------------------------
# Define ping contexts
ping_context_1 = [StateLink(ball, state_var), EqualLink(state_var, neutral)]

# Define ping action
def ping():
    time.sleep(5)
    print("\nJust pinged\n")
    return StateLink(ball, pinged)

ping_action = ExecutionOutputLink(GroundedSchemaNode("py: ping"), ListLink())

# Define ping goal
ping_goal = op.create_goal("ping", 0)

# Define ping-component that uses the default step `psi-step` and default
# action-selector `psi-get-satisfiable-rules`, which are defined in scheme.
ping_component = op.create_component("ping")

# Define ping rules
# The truthvalue for the rule can be used by the action selector for determing
# which action to choose. If you choose to use it when defining your
# action-selector.
op.add_rule(
    ping_context_1, ping_action, ping_goal, TruthValue(1, 1), ping_component
)

# Start ping components
# op.run(ping_component)

# Stop ping components
# op.halt(ping_component)
