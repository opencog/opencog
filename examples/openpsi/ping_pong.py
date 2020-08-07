# This expands on the `ping.py` example by adding a pong component. Thus,
# start by importing defined objects from it.
from ping import *

# Exapnd ball states
ponged = ConceptNode("ponged")

# Expand pinging rules
ping_context_2 = [StateLink(ball, state_var), EqualLink(state_var, ponged)]

op.add_rule(
    ping_context_2, ping_action, ping_goal, TruthValue(1, 1), ping_component
)

# ----------------------------------------------------------------------
# DEFINING PONGING
# ----------------------------------------------------------------------
# Define pong contexts
pong_context = [StateLink(ball, state_var), EqualLink(state_var, pinged)]

# Define pong goal
pong_goal = op.create_goal("pong", 1)

# Define ping action
def pong():
    time.sleep(1)
    print("\nJust ponged\n")
    StateLink(ball, ponged)
    # The side-effect of the action decreases the urge.
    return op.decrease_urge(pong_goal, 1)


pong_action = ExecutionOutputLink(GroundedSchemaNode("py: pong"), ListLink())

# Define pong component that uses custom step in place of `psi-step` and
# default action-selector `psi-get-satisfiable-rules`, which are defined in
# scheme.
def pong_step():
    time.sleep(3)
    urge = op.get_urge(pong_goal)
    if urge < 0.7:
        print("\nNot yet feeling like ponging the ball. Urge = %f\n" % urge)
        op.increase_urge(pong_goal, 0.2)
    else:
        print("\nFeeling like ponging the ball. Urge = %f\n" % urge)
        op.increase_urge(pong_goal, 0.2)
        op.step(ConceptNode("pong"))

    return TruthValue(1, 1)


pong_stepper = EvaluationLink(
    GroundedPredicateNode("py: pong_step"), ListLink()
)

pong_component = op.create_component("pong", pong_stepper)

# Replace the default action-selector of the pong component.
def pong_action_selector():
    return op.get_satisfiable_rules(pong_component)


op.set_action_selector(
    pong_component,
    ExecutionOutputLink(
        GroundedSchemaNode("py: pong_action_selector"), ListLink()
    ),
)

# Define pong rules
op.add_rule(
    pong_context, pong_action, pong_goal, TruthValue(1, 1), pong_component
)

# ----------------------------------------------------------------------
# Start ping and pong components
# op.run(pong_component)
# op.run(ping_component)

# Stop ping and pong components
# op.halt(ping_component)
# op.halt(pong_component)
