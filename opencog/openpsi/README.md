# OpenPsi

OpenPsi is a rule-selection framework intended to provide provide
the top-level behavior control mechanism for robots.  It is loosely
inspired by Joscha Bach's MicroPsi.

Each rule has the abstract form
   if (context) and (action is taken) then (goal is fulfilled).

These rules are classified according to the different demands that
the goals can fulfill.  The main loop is an action-selection mechanism,
examining the current demands, and selecting among the rules that can
satisfy that demand, and also be applicable in the current context.

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

* The demand-update code is undocumented, confusing, and does not
  belong in this directory.


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
            (ImplicationLink  <TV>
                (AndLink
                    (context)
                    (action))
                (goal))
    ```
    An ImplicationLink was choosen because it allows PLN to be employed
    to perform reasoning in the face of uncertain contexts.
  * The function `psi-rule` that is defined [here](main.scm). Is to be used
    in adding new rules.

2. Context:
  * The context part of the rule must be a set of predicates, i.e. atoms
    that can be evaluated, and, when evaluated, return truth values
    (typically a crisp TRUE_TV or FALSE_TV) to indicate if the rules
    is applicable in the current context.  In the current
    implementation, the truth values are assumed to be crisp true/false
    values; thier boolean conjunction is taken to determine the validity
    of the context. The action can then be taken if the resulting truth
    value is strong enough (i.e. is TRUE_TV).

3. Action:
  * The action part of the rule will be evaluated if the rule has
    triggered, i.e. if the rule was selected by the action selector.

    The action can be any evaluatable atom; the intent is that the
    action will alter the atomspace, or send messages to some external
    system.

4. Goal:
  * The goal indicates the degree to which the action, if taken, can
    satisfy the demand or meet the goal.  It can be any atom that,
    when evaluated, returns a truth value.
    (Huh ?? Deamnds are not goals? How do these interact?)

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

## Algorithmic overview:

1. Main loop
  * The `psi-step` function performs a single iteration of the system.
    It performs the steps described below. It is implemented
    [here](main.scm). The main loop just calls the single-step
    function repeatedly.

2. Action selection
  * Action selection chooses one (or more) rules associated with a
    demand. It is implemented in the `psi-select-rules-per-demand`
    function, [here](main.scm).

  * The default action selector finds all satisfiable rules that are
    associated with a demand, and then randomly picks one, according
    to the weight associated with the rule (so that more heavily
    weighted rules are more likely to be picked).

  * A rule is "satisfiable" if the context part of the rule is
    consistent with the current contents of the atomspace. The
    current implementation assumes that satisfiability is a crisp
    true/false value, although it is meant to be fractional, in
    general.

  * The rule weight is currently encoded in the TV in the
    ImplicationLink (see below for details).

  * Each demand can have it's own custom action-selector. The
    user can set these with the `psi-add-action-selector` and
    `psi-action-selector-set!` functions [here](action-selector.scm).

3. Default action selector
  * The default action selector randomly picks one satsifiable rule
    associated with a demand.  If there is more than one rule that
    is satisfiable, then one is selected randomly.

  * The weight of a rule is currently defined as the product of
    the strength and confidence of the ImplicationLink of the rule.

  * The satsifiability of a rule is checked by extracting the context
    and wrapping it in a `SatisfactionLink`, and then evaluating it.
    A rule is satisfiable if the result of evaluation is TRUE_TV.
    See the `psi-satisfiable?` function [here](main.scm).

  * In the future, the weight of a rule will be defined as:
    XXX FIXME: the below is not currently used.
    ```
    Wa = 1/Na * sum ( Wcagi ...)
    Wcagi = Scga * Sc * STIcga
    ```
    where
    Wa = weight of an action
    Wcagi = weight of an action in a single psi-rule 'i' (An action 'a'
            could be part of multiple psi-rules achieving multiple goals,
            so if an action is likely to achieve multiple goals then it
            should have a higher weight). It is implemented in
            `psi-action-weight`.  (Present value =
            Scga * Sc * confidence(cga) * confidence(context))
    Na = Number of rules that have action `a` (Present value =1 )
    Scga = Strength of psi-rule
    Sc = Strength of context that is partially implemented in
        `psi-context-weight`. Implementation details are being discussed
        [here](https://github.com/opencog/atomspace/issues/823).
        (Present value = 0 or 1)
    STIcga = short-term-importance of the psi-rule (Present value = 1)

4. Action Selection bugs. XXX FIXME -- The current implementation has
   multiple implementation issues. These include:

 * If a context contains variables that also appear in the action,
   then these are not handled correctly. For example, if the context
   is "variable $X is flying towards me", and the action is "catch
   variable $X", the current satisfiability code can determine if
   the context holds true, by grounding variable $X, but that
   grounding is not passed to the action. That is, when the action is
   executed, teh variable $X is not known/grounded.

 * The default action selector is extremely inefficient if there
   are more than a few hundred rules, as it attempts to find and
   work with all of them. This overhead can be avoided by using the
   DualLink for narrowing the search for applicable contexts.  There
   is a protottype showing the DualLink in action, in the nlp/aiml
   subsystem. (There are more than 100K AIML rules; the DualLink
   narrows these down to the handful of rules applicable to the
   current context). This code needs to be prted here.

5. Action execution and goal evaluation
  * After a set of rules are selected, then the actions and goals
    attached to these rules are extracted.

  * Given a set of actions, all goals that associated to these
    actions are located (even if those goals belong to rules that
    did not trigger in the current context).  The idea here is that
    performing the action may advance those goals, even though those
    goals were not a part of the selected rules.

  * Related/associated goals are those that are the implicand of the
    psi-rules that have the given action in the implicand. That is, if
    more than one rule has the same action, helping achieve different
    goals, then those goals are related/associated. Thus, the execution
    of the given action should result in acheiving those goals as well,
    regardless of whether the rules that resulted in the association
    are selected or not. See `psi-related-goals` [here](main.scm).

  * The actions are executed, followed by the evaluation of the goals.

  * Bugs: The current implementation fails to check if the action was
    successful, before updating goals.  Just because an action is
    attempted does not imply that the action was acheived. XXX FIXME.


## File overview
* `main.scm` -- Defines the main function for single-stepping the
   psi rule engine, as well as the main-loop to run the stepper.

* `rules.scm` -- Functions for defining openpsi-rules, and fetching
  thier various components.

* `action-selector.scm` -- Implementation of a default action-selector,
  as well as functions that allow a user to specify a custom action
  selector.

* `utilities.scm` -- Miscellaneous utilities.

## OpenPsi examples
* The examples [here](../../examples/openpsi) are currently broken.
* The AIML interpreter [here](../nlp/aiml) uses Opensi and currently
  works.
* The ROS behavior scripts
  [here](http://github.com/opencog/ros-behavior-scripting) use OpenPsi
  and are currently working.

## Modulators and Internal Dynamics

OpenPsi includes a dynamical system of interacting modulator variables.
The model contains varying parameters that regulate processes such as
stimulus evaluation, action selection, and emotional expression.
Modulators are based on MicroPsi, and SECs are based on Component
Process Model theory (see references for  more info). These and other
OpenPsi-related entities dynamically interact with each other according
to rules that specify how a change in a "trigger" entity cause a change
in a "target" entity.

The interaction rules have the logical form of:

    change in trigger entity --> change in target entity

The change in the target is a function of the magnitude of change in the
trigger, the strength of the interaction rule, and the current value of
the target.

The rules in Atomese have the form:

    (PredictiveImplication
        (TimeNode 1)
            (Evaluation
                  (Predicate "psi-changed")
                  (List
                      trigger))
            (ExecutionOutputLink
                (GroundedSchema "scm: adjust-psi-var-level")
                (List
                    target
                    (NumberNode strength)
                    trigger))))

The antecedent predicate can be one of "psi-changed", "psi-increased"
or "psi-decreased".

Files:

* updater.scm - the main control file that handles the dynamic updating
  of entity values based on the interaction rules.
* interaction-rule.scm - code for creating the rules specifying
  interaction dynamics between entities.
* modulator.scm - internal openpsi modulator variables
* sec.scm - internal openpsi Stimulus Evaluation Check variables
* events.scm - defines perceived events that are monitored and that
  trigger changes in openpsi variables.
* emotion.scm - code for creating emotions

Instructions:

* Create interaction rules with the psi-create-interaction-rule function in
  interaction-rule.scm
* Create monitored events with psi-create-monitored-event function in event.scm.
* Create event detection callback(s) with (psi-set-event-callback! callback) in
  updater.scm. The callback function should indicate that a particular event has
  occurred by calling (psi-set-event-occurrence! event), which uses a StateLink
  to represent the most recent occurrence of the event.
* Create "expression" callback function with (psi-set-expression-callback!
  callback) in updater.scm. This function should handle implementation-specific
  expression of emotion and/or physiology based on the OpenPsi dynamic variables.

For a concrete example of using this model, see 
https://github.com/opencog/ros-behavior-scripting/blob/master/src/psi-dynamics.scm

Todo: Create general callback that is called at each loop step.


Open Issues:
------------
 * Can a rule satisfy multiple goals?
 * Can a rule partially satisfy a goal?  How is the partial satisfaction
   indicated?
 * Demand update is confused/confusing
