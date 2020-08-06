# OpenPsi

OpenPsi is a rule-selection framework intended to provide a top-level
behavior control mechanism for robots. It is loosely inspired by
Joscha Bach's MicroPsi.

Each rule has the abstract form
```
   if (context) and (action is taken) then (goal is fulfilled).
```

For examples see [here.](../../examples/openpsi/)

## OpenPsi concepts

**1. Psi-rule:**
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
  * The function `psi-rule`, which is defined in [OpenPsiSCM.cc](OpenPsiSCM.cc),
    must be used for adding new rules, otherwise a valid structured atom
    declared in the atomspace would not be considered as a psi-rule. This is
    because, usage of the function results in the atom being added to
    openpsi's index.

**2. Context:**
  * The context part of the rule must be a set of predicates, i.e. atoms
    that can be evaluated, and, when evaluated, return truth values
    (typically a crisp TRUE_TV or FALSE_TV) to indicate if the rule
    is applicable in the current context. In the current
    implementation, the truth values are assumed to be crisp true/false
    values; their boolean conjunction is taken to determine the satisfiablity
    of the context.
  * The function `psi-satisfiable?`, which is defined in [OpenPsiSCM.cc](OpenPsiSCM.cc),
    is used to check if a psi-rule is satisfiable or not.

**3. Action:**
  * The action part of the rule will be executed if the rule is triggered,
    i.e. if the rule was selected by the action selector. The intent is
    that the action will alter the atomspace, or send messages to some
    external system.
  * The function `psi-imply`, which is defined in [OpenPsiSCM.cc](OpenPsiSCM.cc),
    is used to execute an action of a satisfiable rule.

**4. Goal:**
  * It is an atom used to represent the goal parameter. It has two values, the
    goal-value(gv) which is a measure of the current goal state, and the
    desired-goal-value(dgv) which is preferred goal state.
  * The difference between the values `(gv - dgv)` is called urge.
  * The function `psi-goal`, which is defined in [rule.scm](rule.scm), is used
    to create goals.

**5. Category:**
  * A set of psi-rules are grouped into a psi-category.
  * The function `psi-add-category`, which is defined in [OpenPsiSCM.cc](OpenPsiSCM.cc),
    is used to declare a category. A psi-rule is inserted to a category during
    declartion.

**6. Component:**
  * A component is a psi-rule category, an action-selector associated to
    work on these rules, and the openpsi steps to be executed during a single
    openpsi loop iteration.
  * The function `psi-component`, which is defined in [main.scm](main.scm), is
    used to define a component and it's associated steps.
  * [Ghost](../ghost) is implemented as a openpsi component.

**7. Action-selector:**
  * It is an executable atom the defines how rules in a category should be
    chosen.
  * The function `psi-get-satisfiable-rules`. which is defined in [rule.scm](rule.scm),
    is used to define the default action-selector.
  * To replace the default action-selector of an openpsi component. the
    function `psi-set-action-selector!`, which is defined in
   [action-selector.scm](action-selector.scm), can be used.

**8. Step:**
  * It is a single iteration of openpsi process loop.
  * The function `psi-step`, which is defined in [main.scm](main.scm),
    implements the default step of components. First it performs action
    selection that chooses satisfiable rules, and then it executes the actions
    of these rules.
  * The current code was mainly used for robot control, thus  there isn't any
    goal-value updating taking place, as the goal-values were expected to be
    sensory measurements.
  * To repeatedly run a component's steps in a separate thread use `psi-run`.
    The function `psi-halt` stops the thread. Both function are defined in
    [main.scm](main.scm)

### TODO

* If no exact match to a context can be found, fuzzy matching should
  be performed, to find a context that most closely applies to the
  current situation.

* If no fuzzy match can be found, then a MOSES-inspired mechanism
  should kick in to create new rules.  This includes the selective
  pruning of existing contexts (called "knob-turning" in MOSES),
  exploration of minor elaborations to contexts (also controlled
  by knob-turning) and genetic cross-over.

* The learning mechanism needs to be elaborated into a  "small-data"
  learning system, so that the system can learn from only a handful of
  situations/examples.

## Open Issues

* Can a rule satisfy multiple goals?
* Can a rule partially satisfy a goal?  How is the partial satisfaction
  indicated?
* Also see [github issues](https://github.com/opencog/opencog/issues?q=is%3Aopen+is%3Aissue+label%3Aopenpsi )

## References

* OpenCog Wiki - [OpenPsi](http://wiki.opencog.org/w/OpenPsi)
* Joscha Bach's book - Principles of Synthetic Intelligence
* Wikipedia - [Belief–desire–intention software model](https://en.wikipedia.org/wiki/Belief%E2%80%93desire%E2%80%93intention_software_model) has some
  similarities with respect to demand, goals and actions, that is, has
  the cognitive schematic `Belief + Intention ==> Desire, which is
  similar to the Context + Procedure/Actions ==> Goal`which is used
  here.
