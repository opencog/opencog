; XXX This is a temp hack! Throw away after the demo.
; e.g. Han, can you tell these humans a bit about open clock

(load "contexts.scm")
(load "actions.scm")

(define demo-intro-pattern
    (List
        (Glob "blah-1")
        (Word "you")
        (Word "tell")
        (Glob "blah-2")
        (Word "open")
        (Word "clock"))
)

(define demo-intro-reply
    (map Word (string-split "OpenCog is an open source software framework, aimed toward the creation of Artificial General Intelligence at the human level and ultimately beyond. OpenCog has been used in a variety of areas, including biology, finance, national security, customer support and more. But many of the OpenCog researchers feel that the best way to move OpenCog toward human-level general intelligence is to make OpenCog control humanoid robots, like me!" #\ ))
)

(define (check-for-demo-intro)
    (if (equal? (Set) (cog-execute! (Get (State input-utterance-words demo-intro-pattern))))
        (stv 0 1)
        (stv 1 1)
    )
)

(Define
    (DefinedPredicate "ben-is-asking-for-an-intro?")
    (Evaluation (GroundedPredicate "scm: check-for-demo-intro") (List))
)

(psi-rule
    (list (SequentialAnd
        (DefinedPredicate "is-input-utterance?")
        (DefinedPredicate "ben-is-asking-for-an-intro?")
        (DefinedPredicate "has-not-replied-anything-yet?")
    ))
    (True (ExecutionOutput (GroundedSchema "scm: say") (List demo-intro-reply)))
    (True)
    (stv .9 .9)
    sociality
)
