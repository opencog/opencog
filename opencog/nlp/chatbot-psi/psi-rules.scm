;-------------------------------------------------------------------------------

(define chat-rule (Concept (string-append psi-prefix-str "chat-rule")))

;-------------------------------------------------------------------------------

(Member
    (psi-rule
        (list
            (Not (DefinedPredicate "fuzzy-qa-search-started?"))
            (DefinedPredicate "is-input-utterance?")
            (DefinedPredicate "no-canned-reply?")
            (DefinedPredicate "is-interrogative?")
        )
        (True (ExecutionOutput (GroundedSchema "scm: do-fuzzy-QA") (List)))
        (True)
        (stv 1 1)
        sociality
    )
    chat-rule
)

(Member
    (psi-rule
        (list
            (DefinedPredicate "fuzzy-qa-search-started?")
            (DefinedPredicate "is-fuzzy-answer?")
        )
        (True (ExecutionOutput (GroundedSchema "scm: reply") (List fuzzy-answers)))
        (Evaluation (GroundedPredicate "scm: psi-demand-value-maximize") (List sociality (Number "90")))
        (stv .9 1)
        sociality
    )
    chat-rule
)

(Member
    (psi-rule
        (list
            (Not (DefinedPredicate "aiml-search-started?"))
            (DefinedPredicate "is-input-utterance?")
        )
        (True (ExecutionOutput (GroundedSchema "scm: do-aiml-search") (List)))
        (True)
        (stv 1 1)
        sociality
    )
    chat-rule
)

(Member
    (psi-rule
        (list
            (DefinedPredicate "aiml-search-started?")
            (DefinedPredicate "is-aiml-reply?")
        )
        (True (ExecutionOutput (GroundedSchema "scm: reply") (List aiml-replies)))
        (Evaluation (GroundedPredicate "scm: psi-demand-value-maximize") (List sociality (Number "90")))
        (stv .9 1)
        sociality
    )
    chat-rule
)
