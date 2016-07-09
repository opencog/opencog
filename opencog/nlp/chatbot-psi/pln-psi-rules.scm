;; PLN rules

(psi-rule
    (list (SequentialAnd
        (DefinedPredicate "is-pln-inferred-related?")
        (DefinedPredicate "pln-qa-not-started?")
        (DefinedPredicate "is-input-utterance?")
        (DefinedPredicate "is-a-question?")
    ))
    (True (ExecutionOutput (GroundedSchema "scm: do-pln-QA") (List)))
    (True)
    (stv .95 .9)
    sociality
)

(psi-rule
    (list (SequentialAnd
        (DefinedPredicate "pln-qa-finished?")
        (DefinedPredicate "is-pln-answer?")
        (DefinedPredicate "is-input-utterance?")
    ))
    (True (ExecutionOutput (GroundedSchema "scm: reply") (List pln-answers)))
    (True)
    (stv .95 .9)
    sociality
)
