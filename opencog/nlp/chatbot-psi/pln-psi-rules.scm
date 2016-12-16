;; PLN rules

; If it's in the PLN reasoning mode and the input is a statement, just ack it
(psi-rule
    (list (SequentialAnd
        (DefinedPredicate "is-input-utterance?")
        (DefinedPredicate "is-in-reasoning-mode?")
        (Not (DefinedPredicate "input-is-a-question?"))
        (DefinedPredicate "has-not-replied-anything-yet?")
    ))
    (True (ExecutionOutput (GroundedSchema "scm: ack-the-statement") (List)))
    (True)
    (stv .9 .9)
    sociality
)

; If it's in the PLN reasoning mode and the reasoner doesn't have an answer
; for the question being asked, say "Sorry I don't know"
(psi-rule
    (list (SequentialAnd
        (DefinedPredicate "is-input-utterance?")
        (DefinedPredicate "is-in-reasoning-mode?")
        (DefinedPredicate "input-is-a-question?")
        (SequentialOr
            (Not (DefinedPredicate "is-pln-inferred-related?"))
            (SequentialAnd
                (DefinedPredicate "pln-qa-finished?")
                (Not (DefinedPredicate "is-pln-answer?"))
            ))
        (DefinedPredicate "has-not-replied-anything-yet?")
    ))
    (True (ExecutionOutput (GroundedSchema "scm: say")
        (List (Word "sorry") (Word "I") (Word "don't") (Word "know"))))
    (True)
    (stv .9 .9)
    sociality
)

(psi-set-controlled-rule
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "is-pln-inferred-related?")
            (DefinedPredicate "pln-qa-not-started?")
            (DefinedPredicate "is-input-utterance?")
            (DefinedPredicate "input-is-a-question?")
        ))
        (True (ExecutionOutput (GroundedSchema "scm: do-pln-QA") (List)))
        (True)
        (stv .95 .9)
        sociality
        "select_pln_answer"
    )
)

(psi-set-controlled-rule
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "pln-qa-finished?")
            (DefinedPredicate "is-pln-answer?")
            (DefinedPredicate "has-not-replied-anything-yet?")
        ))
        (True (ExecutionOutput (GroundedSchema "scm: reply") (List pln-answers)))
        (True)
        (stv .95 .9)
        sociality
        "select_pln_answer"
    )
)
