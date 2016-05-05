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
                (AndLink
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
Coming soon :smile:

<!--
See [here](../../examples/openpsi) for some sample implementations of the
framework. -->
### OpenPsi examples
Coming soon :smile:
