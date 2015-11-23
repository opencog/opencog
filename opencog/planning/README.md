## Work In Progress (WIP)

This directory contains code associated with planning.

Presently the definition used here are used by OpenPsi.

### Action selection

1. There are two steps to action selection, they are in no particular order
    * Goal driven action selection
    * Context driven action selection

2. In goal-driven-action-selection, the first step is to select the rules
   depending on a plan, to achieve a goal not accounting for the context. Why
   not account for context? Because there is no guarantee what the context will be during action-execution.

   Following this step, actions that fit context are run. If the intial or any
   subsquent action doesn't fit the goal context then a new goal for creating
   the context is executed or switched to context-driven-action-selection.

3. In context-driven-action-selection, the first step is to select the actions
   that fit the context and do the planning over those actions. Should those
   actions don't achieve the goal, then the planner should return which ever
   sequence of actions comes close.

#### Notes
* The choice of which type of action-selection to run could possible driven
  by an independent agent/planner. In another words, choosing the
  action selection sequence could be in itself a goal.
