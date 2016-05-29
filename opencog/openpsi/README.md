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

### OpenPsi components
1. Psi-rule:
  * A rule is an ImplicationLink structured as
    ```scheme
            (ImplicationLink
                (SequentialAndLink
                    (context)
                    (action))
                (demand-goal))
    ```
    An ImplicationLink was choosen because it allows us to reason and build
    action plans dealing with uncertainty, using PLN rules.
  * The function `psi-rule` that is defined [here](main.scm). Is to be used
    in adding new rules.

2. Context:
  * These are atoms that should be evaluated to return TRUE_TV/FASLE_TV. Should
    all the atoms in the context evaluate to TRUE_TV, then only the actions are
    executed and the goals are evaluated.

3. Action:
  * An action is an `ExecutionOutputLink` or any atom that could be executed by
    running `(cog-execute! your-action)`. An action is the means for interacting
    with the atomspace or other systems.

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
    running `(cog-evaluate! your-goal)`. A goal is the means for manipulating
    the demand-values so as to model emotions/feelings.

5. Demand:
  * A demand is one of the intermediate constructs that are used to model
    emotion/behaviors.
  * The representation is specified in the function `psi-demand` that is
    defined [here](demand.scm). Use this function to create a demand.
  * The demand-value is the stregth of the stv of the ConceptNode. This is used
    for measurement purposes.

6. Modulator and Feeling representation:
Coming soon :smile:

### OpenPsi algorithm
1. psi-step:
  * It wraps all the logic that is described below. And is run once per cycle.
    See function `(psi-step)` which is the entry point [here](main.scm).

2. choose action-selector:
  * If you have defined an action selector then it will be used else the
    default action selector is used. See function `(psi-select-rules)`
    [here](main.scm).
  * Only one action-selector is used in each step for all the demands. See
    `psi-add-action-selector` and `psi-action-selector-set!`
    [here](action-selector.scm) for adding and setting your an action-selector.

3. default action selector:
  * Most-weighted-satisfiable psi-rules of each demand are filtered out. If
    there are more than one of them then one is randomly selected.
  * The weight is calculated as the product of strength and confidence of the
    psi-rule.
  * A rule is checked if is satisfiable by extracting the context and wrapping
    it in a `SatisfactionLink` before evaluating it. If TRUE_TV is returned
    it means that is it satisifiable. See function `psi-satisfiable?`
    [here](main.scm).

4. action execution and goal evaluation:
  * After the rules are choosen then the action and goals are extracted. During
    the goal extraction, every goal that is related through the action are
    choosen.
  * Related/associated goals are those that are the implicand of the psi-rules
    that have the given action in the implicant, that is, if more than one rule
    has the same action helping achieve different goals then those goals are
    related/associated. Thus, the execution of the given action should result
    in achiving those goals as well, regardless of whether the rules that
    resulted in the association are selected or not. See `psi-related-goals`
    [here](main.scm).
  * Then actions are executed followed by the evaluation of the goals. No check
    is made before evaluating of goals to see if the action has been executed.
    It is assumed that if the context is satisfied then there is nothing that
    prevents the action from executing. __This assumption might not work when
    ecan or some other process that modifys the context is running in
    parallel.__

<!--
See [here](../../examples/openpsi) for some sample implementations of the
framework. -->
### OpenPsi examples
Coming soon :smile:
