(psi-rule
    (list (SequentialAnd
        (Not (DefinedPredicate "fuzzy-qa-search-started?"))
        (DefinedPredicate "is-input-utterance?")
        (DefinedPredicate "is-a-question?")
    ))
    (True (ExecutionOutput (GroundedSchema "scm: do-fuzzy-QA") (List)))
    (True)
    (stv .9 .9)
    sociality
)

(psi-rule
    (list (SequentialAnd
        (DefinedPredicate "fuzzy-qa-search-started?")
        (DefinedPredicate "is-fuzzy-answer?")
    ))
    (True (ExecutionOutput (GroundedSchema "scm: reply") (List fuzzy-answers)))
    (True)
    (stv .9 .9)
    sociality
)

(psi-rule
    (list (SequentialAnd
        (Not (DefinedPredicate "fuzzy-match-started?"))
        (DefinedPredicate "is-input-utterance?")
        (Not (DefinedPredicate "is-a-question?"))
    ))
    (True (ExecutionOutput (GroundedSchema "scm: do-fuzzy-match") (List)))
    (True)
    (stv .9 .9)
    sociality
)

(psi-rule
    (list (SequentialAnd
        (DefinedPredicate "fuzzy-match-started?")
        (DefinedPredicate "is-fuzzy-reply?")
    ))
    (True (ExecutionOutput (GroundedSchema "scm: reply") (List fuzzy-replies)))
    (True)
    (stv .9 .9)
    sociality
)

(psi-rule
    (list (SequentialAnd
        (Not (DefinedPredicate "aiml-search-started?"))
        (DefinedPredicate "is-input-utterance?")
    ))
    (True (ExecutionOutput (GroundedSchema "scm: do-aiml-search") (List)))
    (True)
    (stv .9 .9)
    sociality
)

(psi-rule
    (list (SequentialAnd
        (DefinedPredicate "aiml-search-started?")
        (DefinedPredicate "is-aiml-reply?")
    ))
    (True (ExecutionOutput (GroundedSchema "scm: reply") (List aiml-replies)))
    (True)
    (stv .9 .9)
    sociality
)

(psi-rule
    (list (SequentialAnd
        (Not (DefinedPredicate "duckduckgo-search-started?"))
        (DefinedPredicate "is-input-utterance?")
    ))
    (True (ExecutionOutput (GroundedSchema "scm: ask-duckduckgo") (List)))
    (True)
    (stv .9 .9)
    sociality
)

(psi-rule
    (list (SequentialAnd
        (DefinedPredicate "duckduckgo-search-started?")
        (DefinedPredicate "is-duckduckgo-answer?")
    ))
    (True (ExecutionOutput (GroundedSchema "scm: reply") (List duckduckgo-answers)))
    (True)
    (stv .9 .9)
    sociality
)
