;-------------------------------------------------------------------------------

(define chat-rule (Concept (string-append (psi-prefix-str) "chat-rule")))

;-------------------------------------------------------------------------------

(Member
    (psi-rule
        (list (SequentialAnd
            (Not (DefinedPredicate "fuzzy-qa-search-started?"))
            (DefinedPredicate "is-input-utterance?")
            (DefinedPredicate "no-canned-reply?")
            (DefinedPredicate "is-interrogative?")
        ))
        (ExecutionOutput (GroundedSchema "scm: do-fuzzy-QA") (List))
        (True)
        (stv 1 1)
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
        (ExecutionOutput (GroundedSchema "scm: answer") (List fuzzy-answers))
        (Evaluation (GroundedPredicate "scm: psi-demand-value-maximize") (List sociality (Number "90")))
        (stv .9 1)
        sociality
    )
    chat-rule
)
