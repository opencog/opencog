# OpenPsi

OpenPsi is a rule-selection framework intended to provide provide
the top-level behavior control mechanism for robots.  It is loosely
inspired by Joscha Bach's MicroPsi.

Each rule takes the form of if(context) then take(action).  These are
classified into different goals (demands) theat they can fullfil. The
main loop is an action-selection mechanism, examining the context to
see if any of the current demands/goals can be fullfilled, and then
taking apropriate action.

## Status and TODO List

The code currently works at a basic level, and is used to control the
Hanson Robotics robot, coordinating both verbal and non-verbal behavior.
Contexts can be crisp conjunctions of patterns containing variables;
the variables can appear within the actions (where thier grounded values
are used).

## TODO:
* Some of the context-matching code in the nlp/aiml directory should
  be cleaned up, generalized, and moved here.  It attempts to handle
  crisp-valued conjunctions of context with variables.

* If no exact match to a context can be found, fuzzy matching should
  be performed, to find a context that most closely applies to the
  current situation.

* If no fuzzy match can be found, then a MOSES-inspired mechanism
  should kick in to create new rules.  This includes the selective
  pruning of existing contexts (called "knob-turning" in MOSES),
  exploration of minor elaborations to contexts (also controlled
  by knob-turning) and genetic cross-over.

* The above learning mechanism needs to be elaborated into a
  "small-data" learning system, so that the robot can learn
  from only a handful of situations/examples.


### References
* OpenCog Wiki - [OpenPsi](http://wiki.opencog.org/w/OpenPsi)
* Hanson Robotics Wiki - [Emotion modelling](http://wiki.hansonrobotics.com/w/Emotion_modeling)
* [MicroPsi publications](http://micropsi.com/publications/publications.html)
* [MicroPsi source code]()
* Joscha Bach's book -- [Principles of Synthetic Intelligence](http://wiki.humanobs.org/_media/public:events:agi-summerschool-2012:psi-oup-version-draft-jan-08.pdf) (2008 ed.)
* Joscha Bach's book -- [Principles of Synthetic Intelligence](http://micropsi.com/publications/assets/Draft-MicroPsi-JBach-07-03-30.pdf) (2007 ed.)
* Wikipedia - [Belief–desire–intention software model](https://en.wikipedia.org/wiki/Belief%E2%80%93desire%E2%80%93intention_software_model) has some
  similarities with respect to demand, goals and actions, that is, has
  the cognitive schematic `Belief + Intention ==> Desire, which is
  similar to the Context + Procedure/Actions ==> Goal`which is used
  here.

## OpenPsi components
1. Psi-rule:
  * A rule is an ImplicationLink structured as
    ```scheme
            (ImplicationLink
                (SequentialAndLink
                    (context)
                    (action))
                (demand-goal))
    ```
    An ImplicationLink was choosen because it allows PLN to be employed
    to perform reasoning in the face of uncertan contexts.
  * The function `psi-rule` that is defined [here](main.scm). Is to be used
    in adding new rules.

2. Context:
  * The context must be in the form of a set of predicates, i.e. when
    evaluated, these return true/false to indicate if the context
    applies. A conjunction is taken, so that the action is performed
    only if the entire context applies.

3. Action:
  * An action is an `ExecutionOutputLink` or any atom that could be
    executed by running `(cog-execute! your-action)`. An action is the
    means for interacting with the atomspace or other systems.

XXX FIXME the action should be TV-valued, e.g. should probobably be
wrapped inside a TrueLink.  That is because the SequentaialAndLink
assumes that all of its elements are evaluatable; i.e. are TV-valued.
Either that, or we need some other mechanism for grabbing the TV
from the resulting execution of the action.

Note also: the SequentialAnd is used instead of AndLink, because the
AndLink is an unordered link, and thus cannot distinguish its first
and second elements!

4. Goal/Demand-goal:
  * A goal is an `EvaluationLink` or any atom that could be evaluated by
    running `(cog-evaluate! your-goal)`.  The goal indicates the degree
    to which the action, if taken, can satisfy the demand or meet the goal.

5. Demand:
  * A demand is collection of state that must be maximized during
    operation.  That is, the system selects goals (and thus actions)
    in order to maximize the demands placed on the system.
  * Demands can be created using the function `psi-demand`, defined
    [here](demand.scm).
  * The demand-value is the stregth of the TruthValue on the demand
    atom (currently, a ConceptNode by default).

6. Modulator and Feeling representation:
Coming soon :smile:

## OpenPsi algorithm
1. `psi-step`:
  * A function that performs a single iteration of the system. It wraps
    the proceedures described below. It is implemented [here](main.scm).

2. choose action-selector:
  * If you have defined an action selector then it will be used; else the
    default action selector is used. See function `(psi-select-rules)`
    [here](main.scm).
  * Only one action-selector is used in each step, for all demands. See
    `psi-add-action-selector` and `psi-action-selector-set!`
    [here](action-selector.scm) for adding and setting your an action-selector.

3. default action selector:
  * Most-weighted-satisfiable psi-rules of each demand are filtered out. If
    there are more than one of them, then one is randomly selected.
  * The weight is calculated as the product of strength and confidence of the
    psi-rule.
  * The satsifiability of a rule is checked by extracting the context and
    wrapping it in a `SatisfactionLink` before evaluating it. If TRUE_TV is
    returned it means that is it satisifiable. See function `psi-satisfiable?`
    [here](main.scm).

XXX FIXME -- this needs fixing, as contexts may contain variables whose
groundings can carry over into the action.  In addition, it is extremely
inefficient if there are many rules; thus the DualLink can be (should
be) used to narrow down the search for contextts.  A crude prototype of
this has been implemented in the nlp/aiml subsysgtem, and needs to be
generalized and ported over here.

4. action execution and goal evaluation:
  * After the rules are selected, then the action and goals are extracted.
    During the goal extraction, every goal that is related through the
    action are choosen.

  * Related/associated goals are those that are the implicand of the
    psi-rules that have the given action in the implicand. That is, if
    more than one rule has the same action, helping achieve different
    goals, then those goals are related/associated. Thus, the execution
    of the given action should result in acheiving those goals as well,
    regardless of whether the rules that resulted in the association
    are selected or not. See `psi-related-goals` [here](main.scm).

  * Then actions are executed followed by the evaluation of the goals.
    No check is made before evaluating of goals to see if the action
    has suceeded.  That is because it is assumed that if the context
    is satisfied, then there is nothing that prevents the action from
    executing.  _This assumption might not work when ECAN or some other
    process that modifies the context is running in parallel.__

## OpenPsi examples
* The examples [here](../../examples/openpsi) are currently broken.
* The AIML interpreter [here](../nlpe/aiml) uses Opensi and currently
  works.
* The ROS behavior scripts
  [here](http://github.com/opencog/ros-behavior-scripting) use OpenPsi
  and are currently wrking.
