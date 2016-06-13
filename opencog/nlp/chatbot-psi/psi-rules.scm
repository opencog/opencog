;-------------------------------------------------------------------------------

(define chat-rule (Concept (string-append psi-prefix-str "chat-rule")))

;-------------------------------------------------------------------------------

(Member
    (psi-rule
        (list (SequentialAnd
            (Not (DefinedPredicate "fuzzy-qa-search-started?"))
            (DefinedPredicate "is-input-utterance?")
            (DefinedPredicate "is-interrogative?")
        ))
        (True (ExecutionOutput (GroundedSchema "scm: do-fuzzy-QA") (List)))
        (True)
        (stv .9 .9)
        sociality
    )
    chat-rule
)

(Member
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
    chat-rule
)

(Member
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
    chat-rule
)

(Member
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
    chat-rule
)
