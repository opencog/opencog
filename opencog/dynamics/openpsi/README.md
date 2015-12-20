# OpenPsi

OpenPsi is an implementation of Jscha Bach's MicroPsi. It is designed
in such a way that it works well with the OpenCog atomspace
architecture.  It is intended to provide top-level control for robot
behaviors.

### References
* [OpenCog Wiki - OpenPsi](http://wiki.opencog.org/w/OpenPsi_%28Embodiment%29)
* [Hanson Robotics wiki - Emotion modelling](http://wiki.hansonrobotics.com/w/Emotion_modeling)
* [MicroPsi publications](http://micropsi.com/publications/publications.html)
* [MicroPsi source code]()
* Joscha Bach's book -- [Principles of Synthetic Intelligence](http://wiki.humanobs.org/_media/public:events:agi-summerschool-2012:psi-oup-version-draft-jan-08.pdf) (2012 ed.)
* Joscha Bach's book -- [Principles of Synthetic Intelligence](http://micropsi.com/publications/assets/Draft-MicroPsi-JBach-07-03-30.pdf) (2007 ed.)
* [Wikipedia - Belief–desire–intention software model](https://en.wikipedia.org/wiki/Belief%E2%80%93desire%E2%80%93intention_software_model) has some similarities.

Then the cognitive schematic
     Contex & Procedure ==> Goal

### Status
Work is progressing on this, but it is not fully functional. The main
task is refactoring, so as to make it run independently of the
embodiment code. When the refactoring is finalized, it will be possible
to use OpenPsi to drive the goal driven dynamics of mind-agents/process,
embodiment agents as well as other OpenCog modules/components.

### Usage
coming soon

OpenPsi Rules aren't defined yet so nothing interesting occurs.

### How are OpenPsi components represented in AtomSpace?
1. Modulator representation:
For example in the case of Activation Modulator.
```
(InheritanceLink
    ; The strength of the stv is the stimulus level, & confidence is always 1.
    (ConceptNode "OpenPsi: Activation" (stv 0.5 1))
    (ConceptNode "OpenPsi: Modulator")
)

; Stimuli that can act on the psi-modulator Activation. When one wants a process
; to affect the feeling of an agent they must add the function that stimulates
; the modulator.
(EvaluationLink
    (PredicateNode "Psi: Stimulates")
    (ListLink
        ; This is the default function that each psi-modulator must have.
        (GroundedSchemaNode "scm: psi-modulator-updater")
        (ConceptNode "OpenPsi: Activation")
    )
)
```

2. Demand representation:
For example in the case of Affiliation Modulator.
```
(InheritanceLink
    ; The strength of the stv is the demand-value, & confidence is always 1.
    (ConceptNode "OpenPsi: Affiliation" (stv .6 1))
    (ConceptNode "OpenPsi: Demand")
)

(EvaluationLink
    (PredicateNode "must_have_value_within")
    (ListLink
        (ConceptNode "OpenPsi: Affiliation")
        (NumberNode .8)
        (NumberNode 1.)
    )
)

; actions that can act on the psi-demand Competence.
(EvaluationLink
    (PredicateNode "Psi: acts-on")
    (ListLink
        ; This is the default function that each psi-demand must have.
        (GroundedSchemaNode "scm: psi-demand-updater")
        (ConceptNode "OpenPsi: Affiliation")
    )
)
```

3. Goal representation:
coming soon

 * A demand without a goal is meaningless. But a goal could exist without being
associated with a demand. Thus how to represent a goal in a generic way?

 * Can & should a goal exist without being associated with a state/context?

4. A Psi Rule is represented as:
coming soon

### Algorithm
1. What it does?
2. Why it does it that way?
