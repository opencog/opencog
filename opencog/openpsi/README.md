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
For sample implementation of the framework see [here](main.scm).

### How are OpenPsi components represented in Atomese?
1. Demand:
  * The representation is specified in the function `psi-demand-pattern` and
    `define-psi-demand` that is defined [here](demand.scm).
  * Each Demand is a URE rule-base as well. The actions that affect its values
    (demand-values) are a member of the rule-base.
  * It must have one default action-type, during definition, that characterizes
    its behavior(change in demand-value) independent of the other actions that
    could act on it.

2. Action:
  * An action is a URE rule for the demand it affects, with additional atoms
    specifying its effect-type and that it is an opencog-action. See the
    function `define-psi-action` [here](demand.scm).
  * It can have the effect type listed in `(psi-action-types)`

3. Goal:
  * A goal is a demand chosen for a particular action effect-type. See the
    function `psi-select-goal` [here](demand.scm).
  * For goal-selection, you can choose what the criteria for choosing a goal by
    defining a function in a GroundedPredicateNode and passing to
    `psi-select-goal`.


4. OpenPsi's active-schema-pool:
  * This is a separate URE rule-base, and not a demand rule-base. It is
    comprised of actions of demands that are added to it.
  * The choice of actions(action-selection) can be made by specifying a function
    in a GroundedPredicateNode and passing to `psi-select-actions` and for which
    demand it does the selection. See the function `psi-selection-actions` [here](demand.scm).

5. Modulator and Feeling representation:
Coming soon :-)
```
