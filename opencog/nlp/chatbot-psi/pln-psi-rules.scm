;; PLN rules

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
