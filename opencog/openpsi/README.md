# OpenPsi

OpenPsi is an implementation of Jscha Bach's MicroPsi. It is designed
in such a way that it works well with the OpenCog atomspace
architecture.  It is intended to provide top-level control for robot
behaviors.

### References
* OpenCog Wiki - [OpenPsi](http://wiki.opencog.org/w/OpenPsi)
* Hanson Robotics Wiki - [Emotion modelling](http://wiki.hansonrobotics.com/w/Emotion_modeling)
* [MicroPsi publications](http://micropsi.com/publications/publications.html)
* [MicroPsi source code]()
* Joscha Bach's book -- [Principles of Synthetic Intelligence](http://wiki.humanobs.org/_media/public:events:agi-summerschool-2012:psi-oup-version-draft-jan-08.pdf) (2008 ed.)
* Joscha Bach's book -- [Principles of Synthetic Intelligence](http://micropsi.com/publications/assets/Draft-MicroPsi-JBach-07-03-30.pdf) (2007 ed.)
* Wikipedia - [Belief–desire–intention software model](https://en.wikipedia.org/wiki/Belief%E2%80%93desire%E2%80%93intention_software_model) has some similarities with respect to demand, goals and actions,
that is,  the cognitive schematic `Context + Procedure/Actions ==> Goal`
is similar to `Belief + Intention ==> Desire`.

### Usage
See [here](../../examples/openpsi) for some sample implementations of the
framework.

### OpenPsi components
1. Demand:
  * The representation is specified in the function `psi-demand-pattern` and
    `psi-demand` that is defined [here](demand.scm).
  * Each Demand is a URE rule-base as well. The action-rules that affect its
    value (demand-value) are a member of the rule-base.
  * It must have one default action, during definition, that characterizes
    its behavior(aka change in demand-value) independent of the other actions that could act on it. This default action becomes part of the single default
    action-rule for the demand. See [this](example

2. Action:
  * An action is an `ExecutionOutputLink` that forms the implicand of a
    BindLink that forms a URE rule. There actions are pre-defined, and they are
    `psi-action-maximize`, `psi-action-minimize` and `psi-action-keep-range`.
    See [here](demand.scm) for details on these and for defining your own
    action behavior

3. Action-rule:
  * An action-rule is a URE rule for the demand it affects, or is dependent on
    for being driven. In addition it has additional atoms for the purpose of
    specifying its effect-type, and that it is an opencog-action. See the
    function `psi-action-rule` for defining a new action [here](demand.scm).
  * It can have the effect types listed in `(psi-action-types)`.
  * Two action-rules are pre-defined, and they are `psi-action-rule-maximize`,
    and `psi-action-rule-minimize`. See [here](demand.scm).

4. Goal:
  * A goal is a demand chosen for a particular action effect-type. See the
    function `psi-select-random-goal` [here](demand.scm).
  * For goal-selection, you can choose what the criteria for choosing a goal
    should be by defining an evaluatable term using the function
    `psi-add-goal-selector` and setting it to be the goal-selector using the
    function `psi-goal-selector-set!`.

5. OpenPsi's active-schema-pool(asp):
  * This is a separate URE rule-base, that is not a demand rule-base. It is
    comprised of actions of demands that are added to it. The set of
    action-rules thar are member of this rule-base are choosen at run time.
  * The choice of action-rules (aka action-selection) to be member of the asp
    is made by the function `psi-select-action-rules`. You will have to
    specify the action-selctor by using `psi-action-rule-selector-set!`.

6. Modulator and Feeling representation:
Coming soon :-)
